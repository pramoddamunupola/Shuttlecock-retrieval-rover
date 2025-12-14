package com.example.shuttlebot

import android.content.Context
import android.util.AttributeSet
import android.widget.FrameLayout


/**
 * A FrameLayout that maintains a specific aspect ratio.
 */
class AspectRatioFrameLayout @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : FrameLayout(context, attrs, defStyleAttr) {

    // The desired aspect ratio (width / height). Default to 4:3.
    private var aspectRatio = 4.0f / 4.0f

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec)

        val width = measuredWidth
        val height = measuredHeight

        if (width == 0 || height == 0) {
            return
        }

        // Calculate the new height based on the measured width and aspect ratio.
        // We add 0.5f and cast to Int to manually round to the nearest integer.
        // This replaces the .roundToInt() function.
        val newHeight = (width / aspectRatio + 0.5f).toInt()

        // If the new height is different, re-measure the view with the new dimensions
        if (newHeight > 0 && newHeight != height) {
            super.onMeasure(
                MeasureSpec.makeMeasureSpec(width, MeasureSpec.EXACTLY),
                MeasureSpec.makeMeasureSpec(newHeight, MeasureSpec.EXACTLY)
            )
        }
    }
}
