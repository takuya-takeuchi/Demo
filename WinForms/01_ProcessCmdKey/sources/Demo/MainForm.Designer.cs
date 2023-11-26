namespace Demo
{

    partial class MainForm
    {

        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            _labelMessage = new System.Windows.Forms.Label();
            SuspendLayout();
            // 
            // _labelMessage
            // 
            _labelMessage.AutoSize = true;
            _labelMessage.Location = new System.Drawing.Point(12, 9);
            _labelMessage.Name = "_labelMessage";
            _labelMessage.Size = new System.Drawing.Size(38, 15);
            _labelMessage.TabIndex = 0;
            _labelMessage.Text = "";
            // 
            // MainForm
            // 
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            ClientSize = new System.Drawing.Size(520, 39);
            Controls.Add(_labelMessage);
            Name = "MainForm";
            Text = "MainForm";
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private System.Windows.Forms.Label _labelMessage;
    }

}