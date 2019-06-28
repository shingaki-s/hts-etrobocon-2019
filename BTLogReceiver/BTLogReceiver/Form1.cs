using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;

namespace BTLogReceiver
{
    /// <summary>
    /// 参考
    /// https://qiita.com/naokishi/items/bd05e42d3df2c7812fb2
    /// </summary>
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // コンボボックスにシリアルポートの一覧を設定
            this.cmbPort.Items.Clear();
            this.cmbPort.Items.AddRange(
                SerialPort.GetPortNames()
                .OrderBy<string, string>(x => x)
                .ToArray<string>());
        }

        private void BtnConnect_Click(object sender, EventArgs e)
        {
            try
            {
                if (btnConnect.Text == "接続")
                {
                    OpenBluetooth();
                    btnConnect.Text = "切断";
                    btnSend.Enabled = true;
                }
                else
                {
                    CloseBluetooth();
                    btnConnect.Text = "接続";
                    btnSend.Enabled = false;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, this.Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnSend_Click(object sender, EventArgs e)
        {
            try
            {
                SendText(txtSend.Text);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, this.Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void SpBluetooth_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (this.spBluetooth.IsOpen == false)
            {
                return;
            }

            try
            {
                AddRecvLogData(this.spBluetooth.ReadExisting());
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "データ受信エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OpenBluetooth()
        {
            this.spBluetooth.PortName = this.cmbPort.SelectedItem.ToString();
            this.spBluetooth.BaudRate = 7200;
            this.spBluetooth.DataBits = 8;
            this.spBluetooth.Parity = Parity.None;
            this.spBluetooth.StopBits = StopBits.One;
            this.spBluetooth.Encoding = Encoding.UTF8;
            this.spBluetooth.Open();
        }

        private void CloseBluetooth()
        {
            this.spBluetooth.Close();
        }
        private delegate void AddRecvLogDataDelegate(string recv);
        private void AddRecvLogData(string recv)
        {
            if (string.IsNullOrEmpty(recv))
            {
                return;
            }

            if (InvokeRequired)
            {
                Invoke(new AddRecvLogDataDelegate(AddRecvLogData), recv);
                return;
            }

            this.lstLog.Items.Add(recv);
            this.lstLog.SelectedIndex = this.lstLog.Items.Count - 1;
        }

        private void SendText(string text)
        {
            this.spBluetooth.Write(text.TrimEnd());
        }
    }
}
