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
            _ListView = new Demo.CustomListView();
            _ColumnHeader1 = new System.Windows.Forms.ColumnHeader();
            _ColumnHeader2 = new System.Windows.Forms.ColumnHeader();
            _ColumnHeader3 = new System.Windows.Forms.ColumnHeader();
            _ColumnHeader4 = new System.Windows.Forms.ColumnHeader();
            SuspendLayout();
            // 
            // _ListView
            // 
            _ListView.AllowColumnReorder = true;
            _ListView.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] { _ColumnHeader1, _ColumnHeader2, _ColumnHeader3, _ColumnHeader4 });
            _ListView.Dock = System.Windows.Forms.DockStyle.Fill;
            _ListView.FullRowSelect = true;
            _ListView.Location = new System.Drawing.Point(0, 0);
            _ListView.Name = "_ListView";
            _ListView.Size = new System.Drawing.Size(520, 386);
            _ListView.TabIndex = 1;
            _ListView.UseCompatibleStateImageBehavior = false;
            _ListView.View = System.Windows.Forms.View.Details;
            // 
            // _ColumnHeader1
            // 
            _ColumnHeader1.Text = "Header1";
            // 
            // _ColumnHeader2
            // 
            _ColumnHeader2.Text = "Header2";
            // 
            // _ColumnHeader3
            // 
            _ColumnHeader3.Text = "Header3";
            // 
            // _ColumnHeader4
            // 
            _ColumnHeader4.Text = "Header4";
            // 
            // MainForm
            // 
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            ClientSize = new System.Drawing.Size(520, 386);
            Controls.Add(_ListView);
            Name = "MainForm";
            Text = "MainForm";
            ResumeLayout(false);
        }

        #endregion

        private Demo.CustomListView _ListView;
        private System.Windows.Forms.ColumnHeader _ColumnHeader1;
        private System.Windows.Forms.ColumnHeader _ColumnHeader2;
        private System.Windows.Forms.ColumnHeader _ColumnHeader3;
        private System.Windows.Forms.ColumnHeader _ColumnHeader4;
    }

}