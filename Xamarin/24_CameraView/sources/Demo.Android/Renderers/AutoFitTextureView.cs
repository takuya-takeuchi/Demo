using System;

using Android.Content;
using Android.Util;
using Android.Views;

namespace Demo.Droid.Renderers
{

    /// <summary>
    /// Auto fit texture view.
    /// </summary>
    public class AutoFitTextureView : TextureView
    {
        /// <summary>
        /// The width of the m ratio.
        /// </summary>
        private int mRatioWidth = 0;

        /// <summary>
        /// The height of the m ratio.
        /// </summary>
        private int mRatioHeight = 0;

        /// <summary>
        /// Initializes a new instance of the <see cref="T:Demo.Droid.Renderers.AutoFitTextureView"/> class.
        /// </summary>
        /// <param name="context">Context.</param>
        public AutoFitTextureView(Context context)
            : this(context, null)
        {

        }

        /// <summary>
        /// Initializes a new instance of the <see cref="T:Demo.Droid.Renderers.AutoFitTextureView"/> class.
        /// </summary>
        /// <param name="context">Context.</param>
        /// <param name="attrs">Attrs.</param>
        public AutoFitTextureView(Context context, IAttributeSet attrs)
            : this(context, attrs, 0)
        {

        }

        /// <summary>
        /// Initializes a new instance of the <see cref="T:Demo.Droid.Renderers.AutoFitTextureView"/> class.
        /// </summary>
        /// <param name="context">Context.</param>
        /// <param name="attrs">Attrs.</param>
        /// <param name="defStyle">Def style.</param>
        public AutoFitTextureView(Context context, IAttributeSet attrs, int defStyle)
            : base(context, attrs, defStyle)
        {

        }

        /// <summary>
        /// Sets the aspect ratio.
        /// </summary>
        /// <param name="width">Width.</param>
        /// <param name="height">Height.</param>
        public void SetAspectRatio(int width, int height)
        {
            if (width == 0 || height == 0)
            {
                throw new ArgumentException("Size cannot be negative.");
            }

            this.mRatioWidth = width;
            this.mRatioHeight = height;
            this.RequestLayout();
        }

        /// <summary>
        /// Ons the measure.
        /// </summary>
        /// <param name="widthMeasureSpec">Width measure spec.</param>
        /// <param name="heightMeasureSpec">Height measure spec.</param>
        protected override void OnMeasure(int widthMeasureSpec, int heightMeasureSpec)
        {
            base.OnMeasure(widthMeasureSpec, heightMeasureSpec);

            int width = MeasureSpec.GetSize(widthMeasureSpec);
            int height = MeasureSpec.GetSize(heightMeasureSpec);

            //SetMeasuredDimension(width, height);

            if (0 == this.mRatioWidth || 0 == this.mRatioHeight)
            {
                this.SetMeasuredDimension(width, height);
            }
            else
            {
                if (width < (float)height * this.mRatioWidth / (float)this.mRatioHeight)
                {
                    this.SetMeasuredDimension(width, width * this.mRatioHeight / this.mRatioWidth);
                }
                else
                {
                    this.SetMeasuredDimension(height * this.mRatioWidth / this.mRatioHeight, height);
                }
            }
        }
    }
}