package com.example.shuttlebot

import android.graphics.*
import android.os.Bundle
import android.util.Log
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.widget.Button
import android.widget.EditText
import android.widget.ImageView
import android.widget.Toast
import android.widget.ToggleButton
import androidx.appcompat.app.AppCompatActivity
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.nnapi.NnApiDelegate
import java.io.FileInputStream
import java.io.IOException
import java.net.HttpURLConnection
import java.net.URL
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.channels.FileChannel
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

// The different states the rover can be in during auto-mode.
enum class RoverState {
    SEARCHING,    // Looking for a shuttlecock
    ALIGNING,     // Centering a detected shuttlecock
    APPROACHING,  // Moving towards a centered shuttlecock
    COLLECTING    // Performing the final push to collect
}

class MainActivity : AppCompatActivity() {

    private val iouThreshold = 0.4f
    private val TAG = "ShuttleBot"

    // --- Configuration ---
    private val ESP32_CAM_IP = "172.31.43.154"       // Your Camera ESP32 IP
    private val ESP32_CONTROL_IP = "172.31.43.153"  // Your Motor ESP32 IP
    private val streamUrl = "http://$ESP32_CAM_IP/stream"
    private val controlUrlBase = "http://$ESP32_CONTROL_IP"
    // ----------------------

    // --- UI Components ---
    private lateinit var surfaceView: SurfaceView
    private lateinit var overlay: ImageView
    private lateinit var modeSwitch: ToggleButton
    private lateinit var buttonLeft: Button
    private lateinit var buttonRight: Button
    private lateinit var buttonStop: Button
    private lateinit var buttonForward: Button
    private lateinit var buttonReverse: Button
    private lateinit var toggleCollecting: ToggleButton
    private lateinit var editTextKp: EditText
    private lateinit var editTextKi: EditText
    private lateinit var editTextKd: EditText
    private lateinit var buttonSetPid: Button

    // --- TFLite ---
    private lateinit var tflite: Interpreter
    private lateinit var nnApiDelegate: NnApiDelegate
    private lateinit var outputShape: IntArray
    private lateinit var modelOutputBuffer: ByteBuffer
    private val bitmapPaint = Paint(Paint.FILTER_BITMAP_FLAG)
    private val inputSize = 480
    private val scoreThreshold = 0.80f

    // --- Threading & State ---
    private val latestFrameForAI = AtomicReference<Bitmap>()
    private val executor = Executors.newSingleThreadExecutor()
    private val coroutineScope = CoroutineScope(Dispatchers.Main)
    @Volatile
    private var isAutoMode = true
    private var currentState = RoverState.SEARCHING
    private var lastDetectionTime = 0L
    private val SEARCH_TIMEOUT_MS = 3000
    private var lastCommandSentTime = 0L
    private val COMMAND_THROTTLE_MS = 100 // Throttle commands to avoid spamming

    // --- Final Reach Detection ---
    private var lastDetectionY = 0f
    private var lastDetectionCenterX = 0f
    private val BOTTOM_THRESHOLD_PERCENT = 0.75f  // Changed from 0.85 to trigger earlier

    // --- PD Controller Gains & State---
    @Volatile
    private var Kp: Float = 0.4f
    @Volatile
    private var Kd: Float = 2f
    private var lastErrorX: Float = 0f

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Initialize UI components
        surfaceView = findViewById(R.id.surfaceView)
        overlay = findViewById(R.id.overlay)
        modeSwitch = findViewById(R.id.modeSwitch)
        buttonLeft = findViewById(R.id.buttonLeft)
        buttonRight = findViewById(R.id.buttonRight)
        buttonStop = findViewById(R.id.buttonStop)
        buttonForward = findViewById(R.id.buttonForward)
        buttonReverse = findViewById(R.id.buttonReverse)
        toggleCollecting = findViewById(R.id.toggleCollecting)
        editTextKp = findViewById(R.id.editTextKp)
        editTextKi = findViewById(R.id.editTextKi)
        editTextKd = findViewById(R.id.editTextKd)
        buttonSetPid = findViewById(R.id.buttonSetPid)

        // Set Initial PID Values in UI
        editTextKp.setText(Kp.toString())
        editTextKi.setText("0.01")
        editTextKd.setText(Kd.toString())

        // Setup listeners
        buttonForward.setOnClickListener { sendControlCommand("/move?dir=forward") }
        buttonReverse.setOnClickListener { sendControlCommand("/move?dir=reverse") }
        buttonLeft.setOnClickListener { sendControlCommand("/move?dir=left") }
        buttonRight.setOnClickListener { sendControlCommand("/move?dir=right") }
        buttonStop.setOnClickListener { sendControlCommand("/stop") }

        toggleCollecting.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) {
                sendControlCommand("/on_collecting")
            } else {
                sendControlCommand("/off_collecting")
            }
        }

        modeSwitch.setOnCheckedChangeListener { _, isChecked ->
            isAutoMode = isChecked
            if (!isChecked) {
                currentState = RoverState.SEARCHING
            }
            updateControlState()
        }

        buttonSetPid.setOnClickListener {
            val kpValue = editTextKp.text.toString().toFloatOrNull()
            val kdValue = editTextKd.text.toString().toFloatOrNull()

            if (kpValue != null) {
                this.Kp = kpValue
                Log.d(TAG, "Controller Kp updated to: ${this.Kp}")
            }
            if (kdValue != null) {
                this.Kd = kdValue
                Log.d(TAG, "Controller Kd updated to: ${this.Kd}")
            }

            if(kpValue != null || kdValue != null) {
                Toast.makeText(this, "Gains updated: Kp=${this.Kp}, Kd=${this.Kd}", Toast.LENGTH_SHORT).show()
            } else {
                Toast.makeText(this, "Invalid Kp or Kd value", Toast.LENGTH_SHORT).show()
            }
        }

        updateControlState()
        setupInterpreter()
        startCombinedStream(streamUrl)
    }

    private fun updateControlState() {
        val isManualMode = !isAutoMode
        buttonLeft.isEnabled = isManualMode
        buttonRight.isEnabled = isManualMode
        buttonStop.isEnabled = isManualMode
        buttonForward.isEnabled = isManualMode
        buttonReverse.isEnabled = isManualMode
        modeSwitch.isChecked = isAutoMode

        if (isManualMode) {
            sendControlCommand("/stop")
        }
    }

    private fun setupInterpreter() {
        val options = Interpreter.Options().apply {
            nnApiDelegate = NnApiDelegate()
            addDelegate(nnApiDelegate)
        }
        try {
            tflite = Interpreter(loadModelFile("yolo.tflite"), options)
        } catch (e: Exception) {
            Log.e(TAG, "NNAPI Delegate failed, falling back to CPU.", e)
            tflite = Interpreter(loadModelFile("yolo.tflite"))
        }

        val outputTensor = tflite.getOutputTensor(0)
        outputShape = outputTensor.shape()
        val outputSize = outputShape[0] * outputShape[1] * outputShape[2] * 4
        modelOutputBuffer = ByteBuffer.allocateDirect(outputSize).order(ByteOrder.nativeOrder())
    }

    private fun loadModelFile(modelName: String): ByteBuffer {
        try {
            val fileDescriptor = assets.openFd(modelName)
            val inputStream = FileInputStream(fileDescriptor.fileDescriptor)
            val fileChannel = inputStream.channel
            val startOffset = fileDescriptor.startOffset
            val declaredLength = fileDescriptor.declaredLength
            val mappedByteBuffer = fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength)
            return mappedByteBuffer
        } catch (e: IOException) {
            Log.e(TAG, "Error loading model file: $modelName", e)
            throw RuntimeException("Error loading model file, app cannot start.", e)
        }
    }

    private fun startCombinedStream(urlStr: String) {
        // Video reading thread
        Thread {
            try {
                val url = URL(urlStr)
                val mjpegStream = MjpegInputStream(url.openConnection().getInputStream())
                while (true) {
                    val frame = mjpegStream.readFrame() ?: break
                    latestFrameForAI.set(frame)
                    drawVideoFrame(frame)
                }
            } catch (e: Exception) {
                if (e !is InterruptedException) {
                    Log.e(TAG, "Video Stream Error: ${e.message}", e)
                }
            }
        }.start()

        // AI processing thread with State Machine
        executor.execute {
            while (!Thread.currentThread().isInterrupted) {
                try {
                    val frameToProcess = latestFrameForAI.get()
                    if (frameToProcess != null && isAutoMode) {
                        val detections = runObjectDetection(frameToProcess)
                        val bestDetection = detections.maxByOrNull { it.box.bottom }

                        drawDetectionOverlay(detections)

                        // --- AUTO MODE STATE MACHINE ---
                        val screenCenterX = overlay.width / 2f
                        val screenHeight = overlay.height.toFloat()
                        val bottomThreshold = screenHeight * BOTTOM_THRESHOLD_PERCENT

                        val isShuttleDetected = bestDetection != null
                        val wasDetectedRecently = System.currentTimeMillis() - lastDetectionTime < 200

                        // Check if shuttlecock was centered and at bottom before disappearing
                        val wasShuttleCenteredAtBottom = if (bestDetection != null) {
                            val errorX = bestDetection.box.centerX() - screenCenterX
                            val isAtBottom = bestDetection.box.bottom > bottomThreshold
                            val isCentered = abs(errorX) < 75
                            isAtBottom && isCentered
                        } else {
                            val errorX = lastDetectionY - screenCenterX
                            lastDetectionY > bottomThreshold && abs(errorX) < 75
                        }

                        // Track Y position and X position for centered shuttlecocks
                        if (bestDetection != null) {
                            val errorX = bestDetection.box.centerX() - screenCenterX
                            if (abs(errorX) < 75) {
                                lastDetectionY = bestDetection.box.centerY()
                                lastDetectionCenterX = bestDetection.box.centerX()
                            }
                        }

                        // STATE TRANSITION LOGIC
                        val previousState = currentState
                        when (currentState) {
                            RoverState.SEARCHING -> {
                                if (isShuttleDetected) {
                                    sendControlCommand("/stop")
                                    currentState = RoverState.ALIGNING
                                    lastDetectionY = 0f
                                    lastDetectionCenterX = 0f  // Reset tracking
                                }
                            }
                            RoverState.ALIGNING -> {
                                if (!isShuttleDetected) {
                                    if (System.currentTimeMillis() - lastDetectionTime > SEARCH_TIMEOUT_MS) {
                                        currentState = RoverState.SEARCHING
                                    }
                                } else {
                                    lastDetectionTime = System.currentTimeMillis()
                                    val errorX = bestDetection!!.box.centerX() - screenCenterX
                                    if (abs(errorX) < 40) {
                                        currentState = RoverState.APPROACHING
                                    }
                                }
                            }
                            RoverState.APPROACHING -> {
                                if (isShuttleDetected) {
                                    lastDetectionTime = System.currentTimeMillis()
                                    val errorX = bestDetection!!.box.centerX() - screenCenterX
                                    if (abs(errorX) >= 40) {
                                        currentState = RoverState.ALIGNING
                                    }
                                    // Check if shuttlecock is at the bottom while still visible
                                    else if (wasShuttleCenteredAtBottom) {
                                        currentState = RoverState.COLLECTING
                                        Log.d(TAG, "Shuttlecock at bottom of screen - FINAL REACH")
                                    }
                                } else if (wasDetectedRecently) {
                                    // Lost detection but was recently seen
                                    if (wasShuttleCenteredAtBottom) {
                                        currentState = RoverState.COLLECTING
                                        Log.d(TAG, "Shuttlecock disappeared from bottom center - FINAL REACH")
                                    } else {
                                        // Lost but not at bottom - keep approaching briefly
                                        if (System.currentTimeMillis() - lastDetectionTime > 500) {
                                            currentState = RoverState.SEARCHING
                                        }
                                    }
                                } else {
                                    currentState = RoverState.SEARCHING
                                }
                            }
                            RoverState.COLLECTING -> {
                                currentState = RoverState.SEARCHING
                                lastDetectionY = 0f
                                lastDetectionCenterX = 0f  // Reset tracking
                            }
                        }

                        if (currentState != previousState) {
                            Log.d(TAG, "State Change: $previousState -> $currentState")
                        }

                        // ACTION LOGIC with PD Controller
                        when (currentState) {
                            RoverState.SEARCHING -> {
                                sendControlCommand("/move?dir=left&speed=220")

                                lastErrorX = 0f
                            }
                            RoverState.ALIGNING -> {
                                if (bestDetection != null) {
                                    val errorX = bestDetection.box.centerX() - screenCenterX
                                    val deadZone = 40

                                    if (abs(errorX) < deadZone) {
                                        sendControlCommand("/stop")
                                        Log.d(TAG, "Action: ALIGNING. Centered. Stopping.")
                                        lastErrorX = 0f
                                    } else {
                                        // PD CONTROLLER LOGIC
                                        val p_speed = abs(errorX) * this.Kp
                                        val errorDelta = errorX - lastErrorX
                                        val d_effect = abs(errorDelta * this.Kd)
                                        var speed = (p_speed - d_effect).toInt()

                                        val direction = if (errorX < 0) "left" else "right"
                                        val maxSpeed = 255f
                                        val minTurnSpeed = 230
                                        if (speed in 1 until minTurnSpeed) {
                                            speed = minTurnSpeed
                                        }
                                        speed = speed.coerceIn(0, maxSpeed.toInt())

                                        sendControlCommand("/turn?dir=$direction&speed=$speed")
                                        Log.d(TAG, "Action: ALIGNING. Dir: $direction, Speed: $speed (P:${p_speed.toInt()}, D:${d_effect.toInt()})")
                                        lastErrorX = errorX
                                    }
                                }
                            }
                            RoverState.APPROACHING -> {
                                // Only send forward command if enough time has passed (throttle)
                                val currentTime = System.currentTimeMillis()
                                if (currentTime - lastCommandSentTime > COMMAND_THROTTLE_MS) {
                                    sendControlCommand("/move?dir=forward")
                                    lastCommandSentTime = currentTime
                                    Log.d(TAG, "Action: APPROACHING. Moving FORWARD.")
                                }
                                lastErrorX = 0f
                                // No sleep here - keep processing fast to detect when it disappears
                            }
                            RoverState.COLLECTING -> {
                                sendControlCommand("/final_reach")
                                Log.d(TAG, "Action: COLLECTING (/final_reach sent - 4s push)")
                                lastErrorX = 0f
                                Thread.sleep(4100) // Wait for final reach to complete (4s + buffer)
                            }
                        }
                    } else if (frameToProcess != null && !isAutoMode) {
                        val detections = runObjectDetection(frameToProcess)
                        drawDetectionOverlay(detections)
                    }
                } catch (e: InterruptedException) {
                    Thread.currentThread().interrupt()
                }
                // Reduce sleep time for faster detection processing during approach
                val sleepTime = if (isAutoMode && currentState == RoverState.APPROACHING) 20 else 50
                Thread.sleep(sleepTime.toLong())
            }
        }
    }

    private fun drawVideoFrame(frame: Bitmap) {
        if (!surfaceView.holder.surface.isValid) return
        val holder: SurfaceHolder = surfaceView.holder
        val canvas: Canvas? = holder.lockCanvas()
        if (canvas != null) {
            try {
                val destRect = Rect(0, 0, canvas.width, canvas.height)
                canvas.drawBitmap(frame, null, destRect, null)
            } finally {
                holder.unlockCanvasAndPost(canvas)
            }
        }
    }

    private fun runObjectDetection(frame: Bitmap): List<Detection> {
        val letterboxedBitmap = Bitmap.createBitmap(inputSize, inputSize, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(letterboxedBitmap)
        canvas.drawColor(Color.BLACK)

        val aspectRatio = frame.width.toFloat() / frame.height.toFloat()
        val destRect: RectF
        if (aspectRatio > 1) {
            val newHeight = inputSize / aspectRatio
            val top = (inputSize - newHeight) / 2
            destRect = RectF(0f, top, inputSize.toFloat(), top + newHeight)
        } else {
            val newWidth = inputSize * aspectRatio
            val left = (inputSize - newWidth) / 2
            destRect = RectF(left, 0f, left + newWidth, inputSize.toFloat())
        }
        canvas.drawBitmap(frame, null, destRect, bitmapPaint)

        val inputBuffer = ByteBuffer.allocateDirect(4 * inputSize * inputSize * 3).apply {
            order(ByteOrder.nativeOrder())
            val pixels = IntArray(inputSize * inputSize)
            letterboxedBitmap.getPixels(pixels, 0, inputSize, 0, 0, inputSize, inputSize)
            for (pixelValue in pixels) {
                putFloat(((pixelValue shr 16) and 0xFF) / 255f)
                putFloat(((pixelValue shr 8) and 0xFF) / 255f)
                putFloat((pixelValue and 0xFF) / 255f)
            }
            rewind()
        }

        modelOutputBuffer.rewind()
        tflite.run(inputBuffer, modelOutputBuffer)

        return parseDetections()
    }

    private fun parseDetections(): List<Detection> {
        modelOutputBuffer.rewind()
        val floatBuffer = modelOutputBuffer.asFloatBuffer()

        val initialDetections = mutableListOf<Detection>()
        if (overlay.width == 0) return initialDetections
        val scaleX = overlay.width.toFloat()
        val scaleY = overlay.height.toFloat()

        val numBoxes = outputShape[2]

        for (i in 0 until numBoxes) {
            val score = floatBuffer[4 * numBoxes + i]
            if (score < scoreThreshold) continue

            val x = floatBuffer[0 * numBoxes + i] * scaleX
            val y = floatBuffer[1 * numBoxes + i] * scaleY
            val w = floatBuffer[2 * numBoxes + i] * scaleX
            val h = floatBuffer[3 * numBoxes + i] * scaleY

            initialDetections.add(Detection(RectF(x - w / 2, y - h / 2, x + w / 2, y + h / 2), score))
        }

        return nonMaxSuppression(initialDetections, iouThreshold)
    }

    private fun nonMaxSuppression(detections: List<Detection>, iouThreshold: Float): List<Detection> {
        val sortedDetections = detections.sortedByDescending { it.score }
        val selectedDetections = mutableListOf<Detection>()
        val active = sortedDetections.toMutableList()

        while (active.isNotEmpty()) {
            val bestDetection = active.first()
            selectedDetections.add(bestDetection)
            active.removeAt(0)

            val boxesToRemove = active.filter { calculateIoU(bestDetection.box, it.box) > iouThreshold }
            active.removeAll(boxesToRemove)
        }
        return selectedDetections
    }

    private fun calculateIoU(box1: RectF, box2: RectF): Float {
        val xA = max(box1.left, box2.left)
        val yA = max(box1.top, box2.top)
        val xB = min(box1.right, box2.right)
        val yB = min(box1.bottom, box2.bottom)

        val intersectionArea = max(0f, xB - xA) * max(0f, yB - yA)
        val box1Area = (box1.right - box1.left) * (box1.bottom - box1.top)
        val box2Area = (box2.right - box2.left) * (box2.bottom - box2.top)
        val unionArea = box1Area + box2Area - intersectionArea

        val iou = intersectionArea / unionArea
        return if (iou.isNaN()) 0f else iou
    }

    private fun drawDetectionOverlay(detections: List<Detection>) {
        if (overlay.width == 0) return
        val overlayBitmap = Bitmap.createBitmap(overlay.width, overlay.height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(overlayBitmap)

        val paint = Paint().apply {
            color = Color.RED
            style = Paint.Style.STROKE
            strokeWidth = 4f
        }
        val textPaint = Paint().apply {
            color = Color.WHITE
            textSize = 40f
        }

        for (d in detections) {
            canvas.drawRect(d.box, paint)
            canvas.drawText("obj: ${"%.2f".format(d.score)}", d.box.left, max(0f, d.box.top - 10f), textPaint)
        }

        runOnUiThread {
            overlay.setImageBitmap(overlayBitmap)
        }
    }

    private fun sendControlCommand(path: String) {
        coroutineScope.launch(Dispatchers.IO) {
            var connection: HttpURLConnection? = null
            try {
                val url = URL("$controlUrlBase$path")
                connection = (url.openConnection() as HttpURLConnection).apply {
                    requestMethod = "GET"
                    connectTimeout = 500
                    readTimeout = 500
                    connect()
                    if (responseCode != 200) {
                        Log.w(TAG, "Command: ${url.path}, Response: $responseCode")
                    }
                }
                connection.inputStream.close()
            } catch (e: Exception) {
                if (e !is java.net.SocketTimeoutException) {
                    Log.e(TAG, "Failed to send command '$path': ${e.message}")
                }
            } finally {
                connection?.disconnect()
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        coroutineScope.cancel()
        executor.shutdownNow()
        if (::tflite.isInitialized) tflite.close()
        if (::nnApiDelegate.isInitialized) nnApiDelegate.close()
    }

    data class Detection(val box: RectF, val score: Float)
}