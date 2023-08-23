using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Runtime.Serialization;
using System.Reflection.PortableExecutable;
using System.Windows.Threading;
using System.Globalization;

namespace A72
{

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
    
    public partial class MainWindow : Window
    {
        private System.ComponentModel.BackgroundWorker bw = new BackgroundWorker() ;
        private static string Runing_Batch= System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "algorithm","run.bat");
        private static string log_main_out = "";
        private static string log_main_err = "";
        private string logFilePath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output_log.txt");
        private string errorFilePath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output_log.txt");
        private string args_add="";
        private string SN;
        private string OperateID;
        private int percent = 0;
        private string txt_path = AppDomain.CurrentDomain.BaseDirectory+ "UI_status.txt";
        private string main_path = AppDomain.CurrentDomain.BaseDirectory;
        public result A72_result;
        public UI A72_UI;
        private DispatcherTimer? dispatcherTimer;
        private int DuringTime;
        private TestItems CurItem;
        


        public MainWindow()
        {
            InitializeComponent();
            initBackgroundWorker();
            SN = "None";
            OperateID = "None";
            A72_UI =new UI();
            //A72_UI.Load_UI_StatusFileAsync();
            this.messenage.Text = A72_UI.message_show;
            dataGrid.DataContext = A72_UI.memberData;
            A72_result = new result();
        }

        void DataWindow_Closing(object sender, CancelEventArgs e)
        {
            
            
            string msg = "Close without saving?";
            MessageBoxResult result =
                MessageBox.Show(
                msg,
                "Data App",
                MessageBoxButton.YesNo,
                MessageBoxImage.Warning);
            if (result == MessageBoxResult.No)
            {
                // If user doesn't want to close, cancel closure
                e.Cancel = true;
            }
            if (result == MessageBoxResult.Yes)
            {
                // If user doesn't want to close, cancel closure
                Environment.Exit(0);
            }
            
        }
        public void ShowCurrentTime(object? sender, EventArgs e)
        {

            //this.time_calculate.Text = DateTime.Now;
            DuringTime+=1;
            this.time_calculate.Text = DuringTime.ToString()+"s";
            //DateTime.Now.ToString("HH:mm:ss");
        }
        private void readtxt()
        {
            //Console.WriteLine("debug");
            String? line;
            String status;

                //Pass the file path and file name to the StreamReader constructor
            StreamReader sr = new StreamReader(txt_path);
                
            //Read the first line of text
            line = sr.ReadLine();
            //Continue to read until you reach end of file
            while (line != null)
            {
                //write the lie to console window
                //Console.WriteLine(line);
                //Read the next line
                if (line.Contains("image imfo."))
                {
                    status=line.Split(':')[1];
                    this.image_message.Content = status;
                }
                line = sr.ReadLine();
            }
            //close the file
            sr.Close();
        }
        private void initBackgroundWorker()
        {
            bw = new BackgroundWorker();
            bw.WorkerReportsProgress = true;
            bw.WorkerSupportsCancellation = true;
            bw.DoWork += new DoWorkEventHandler(bw_DoWork);
            bw.ProgressChanged += new ProgressChangedEventHandler(bw_ProgressChanged);
            bw.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
            
      
        }
        private void bw_DoWork(object? sender, DoWorkEventArgs e)
        {
            try
            {
                logFilePath = System.IO.Path.Combine(main_path, "output_log.txt");
                errorFilePath = System.IO.Path.Combine(main_path, "output_error.txt");
                string args = "-S " + SN + " -O " + OperateID + " -D " + main_path + args_add;
                Console.WriteLine(AppDomain.CurrentDomain.BaseDirectory);
                bw.ReportProgress(0);
                StartMainProgram(args);
                Console.WriteLine("end");
                
                
            }
            catch (Exception ex)
            {
                if (bw.WorkerSupportsCancellation)
                {
                    bw.CancelAsync();
                }
                Console.WriteLine("An error occurred: " + ex.Message);
                // 這裡你可以根據需要採取適當的措施，例如取消背景工作或執行後續處理
            }

        }
        private byte[] ReadImageFromPython(string imagePath)
        {


            // 範例中直接從檔案讀取圖片的二進位數據
            byte[] imageBytes = File.ReadAllBytes(imagePath);
            return imageBytes;
        }
        private async void bw_ProgressChanged(object? sender, ProgressChangedEventArgs e)
        {
            progressBar1.Value = e.ProgressPercentage;
            //this.lblMsg.Text = e.ProgressPercentage.ToString();
            messenage.Text = "Process" + e.ProgressPercentage.ToString()+"%";
            if (File.Exists(AppDomain.CurrentDomain.BaseDirectory+"UI.png") == true)
            {
                //Uri fileUri = new Uri(@"C:/Users/11011105/google/quanta/UI/A72/A72/bin/Debug/net6.0-windows/show.png", UriKind.Relative);
                //imagebox.Source = new BitmapImage(fileUri);
                
                while (true)
                {
                    readtxt();
                    try
                    {
                        byte[] imageBytes = ReadImageFromPython(AppDomain.CurrentDomain.BaseDirectory + "UI.png");

                        // 將二進位數據轉換為 BitmapImage
                        BitmapImage bitmap = new BitmapImage();
                        bitmap.BeginInit();
                        bitmap.StreamSource = new MemoryStream(imageBytes);
                        bitmap.EndInit();

                        // 設定 Image 控件的 Source 屬性
                        imagebox.Source = bitmap;
                        
                        File.Delete(AppDomain.CurrentDomain.BaseDirectory + "UI.png");
                        break;
                    }
                    catch
                    {
                        // 圖片載入失敗，等待1秒再重試
                        await Task.Delay(1000);
                    }
                }
            }
            else
            {
                image_message.Content = "image not found";
            }
        }


        private void bw_RunWorkerCompleted(object? sender, RunWorkerCompletedEventArgs e)
        {

            if ((e.Cancelled == true))
            {
                this.messenage.Text = "cancel !";
            }

            else if (!(e.Error == null))
            {
                this.messenage.Text = ("Error: " + e.Error.Message);
                this.progressBar1.Value = 0;
                if (dispatcherTimer != null)
                {
                    dispatcherTimer.Stop();
                }
            }

            else
            {
                if (dispatcherTimer!=null)
                {
                    dispatcherTimer.Stop();
                }
                //this.memberData[]
                if(A72_UI.error_bool==false)
                {
                    this.messenage.Text = "completed!";
                    this.progressBar1.Value = 100;
                }
                else
                {
                    this.messenage.Text = A72_UI.message_show;
                    Console.WriteLine("Error");
                    this.progressBar1.Value = 0;
                }
                
            }
        }

        private bool StartMainProgram(string allArgs)
        {


            log_main_out = "";
            log_main_err = "";
            ProcessStartInfo startInfo = new ProcessStartInfo(Runing_Batch);
            startInfo.Arguments = allArgs;
            startInfo.WorkingDirectory = ".\\";
            startInfo.WindowStyle = ProcessWindowStyle.Minimized;
            startInfo.UseShellExecute = false;
            startInfo.RedirectStandardOutput = true;
            startInfo.RedirectStandardError = true;

            Process? p = Process.Start(startInfo);
            if (CurItem == TestItems.ALL)
            {
                A72_result.deleteCsv();//刪除舊資料
            }
            
            if (p != null)
            {
                p.OutputDataReceived += build_OutputDataReceived;
                p.ErrorDataReceived += build_ErrorDataReceived;

                p.BeginOutputReadLine();
                p.BeginErrorReadLine();

                p.WaitForExit();
            }

            using (StreamWriter output_log = new StreamWriter(logFilePath))
            {
                output_log.WriteLine(log_main_out);
            }
            using (StreamWriter output_err = new StreamWriter(errorFilePath))
            {
                output_err.WriteLine(log_main_out);
            }
            bool b = (log_main_err.Contains("error") || log_main_err.Contains("Error"));
                return (!b);

        }
        void build_OutputDataReceived(object sender, DataReceivedEventArgs e)
        {
            log_main_out += e.Data;
            log_main_out += "\n";
            Console.WriteLine(e.Data);
            if (e.Data != null)
            {
                if (e.Data.Contains("Status update:"))
                {
                    A72_UI.Load_UI_StatusFile();
                    if (e.Data.Contains("Done"))
                    {
                        percent += 2;
                        bw.ReportProgress(percent);
                    }
                }
                if (e.Data.Contains("show UI image"))
                {
                    percent += 4;
                    bw.ReportProgress(percent);
                }
                if (e.Data.Contains("A72 ERROR:"))
                {
                    percent += 2;
                    A72_UI.message_show= e.Data.Substring(11);
                    A72_UI.error_bool = true;
                }
            }

        }
        void build_ErrorDataReceived(object sender, DataReceivedEventArgs e)
        {
            log_main_err += e.Data;
            log_main_err += "\n";
            Console.WriteLine(e.Data);
        }
        private void Button_Click(object sender, RoutedEventArgs e)
        {
            percent = 0;
            SN = TextBox_SN.Text;
            OperateID =TextBox_OperateID.Text;
            main_path = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory ,"result", SN + "\\");
            Change_Args();


            if (Directory.Exists(main_path))
            {
                Console.WriteLine("The directory {0} already exists.", main_path);
            }
            else
            {
                Directory.CreateDirectory(main_path);
                Console.WriteLine("The directory {0} was created.", main_path);
            }
            if (TextBox_SN.Text!="" && TextBox_OperateID.Text!="")
            {
                DuringTime = 0;
                dispatcherTimer = new System.Windows.Threading.DispatcherTimer();
                // 当间隔时间过去时发生的事件
                dispatcherTimer.Tick += new EventHandler(ShowCurrentTime);
                dispatcherTimer.Interval = new TimeSpan(0, 0, 0, 1);
                dispatcherTimer.Start();
                bw.RunWorkerAsync();
            }
            else
            { MessageBox.Show("Please key in SN and OperateID,thanks"); }
            //progressBar1.Value+= 10;
        }

        private void Show_data_Click(object sender, RoutedEventArgs e)
        {
            var window = new Window1();

            window.Owner = this;
            window.Show();
            
            A72_result.LoadCsvFileAsync();
            window.CTF_result_list.DataContext =A72_result.CTF_Data ;
            this.messenage.Text = "CTF window showing";
        }
        private void Show_Gamma_data_Click(object sender, RoutedEventArgs e)
        {
            var window = new Gamma();

            window.Owner = this;
            window.Show();
            //A72_result = new result();
            //A72_result.LoadGammaCsvFileAsync();
            //window.Gamma_result_list.DataContext = A72_result.Gamma_Data_white;
            //window.Gamma_distort_data.DataContext = A72_result.Gamma_Distort_white;
            this.messenage.Text = "Gamma window showing";
        }
        private void Change_Args()
        {
            switch (ComboBox.SelectedIndex)
            {
                case 0:
                    CurItem = TestItems.ALL;
                    args_add = " -C -B -U -Con";
                    break;
                case 1:
                    CurItem = TestItems.Brightness;
                    args_add = " -B";
                    break;
                case 2:
                    CurItem = TestItems.CTF;
                    args_add = " -C";
                    break;
                case 3:
                    CurItem = TestItems.Contrast;
                    args_add = " -Con";
                    break;
                case 4:
                    CurItem = TestItems.Uniformity;
                    args_add = " -U";
                    break;
            }
        }
        private void ComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

            Change_Args();
            this.messenage.Text = "Switch to " + Enum.GetName(typeof(TestItems), CurItem) + " test~";
        }

        private void Show_Result_Folder(object sender, RoutedEventArgs e)
        {
            string filename = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "result");
            Process.Start("Explorer.exe", $"/e, {filename}");
        }

        private void Show_DUT_Folder(object sender, RoutedEventArgs e)
        {
            string filename = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "result", TextBox_SN.Text+"\\");
            Process.Start("Explorer.exe", $"/e, {filename}");
        }

        private void Load_pos_Click(object sender, RoutedEventArgs e)
        {
            args_add = " -P Load";
            bw.RunWorkerAsync();
            
        }

        private void Test_pos_Click(object sender, RoutedEventArgs e)
        {
            args_add = " -P Test";
            bw.RunWorkerAsync();
            
        }
    }
    public enum TestItems
    {
    ALL,
    CTF,
    Brightness,
    Contrast,
    Uniformity
    }


}
