using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Win32;
using System.Diagnostics;
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
using System.Runtime.CompilerServices;

namespace A72
{
    public class result
    {
        public ObservableCollection<CTF> CTF_Data = new ObservableCollection<CTF>();
        public ObservableCollection<Gamma_result> Gamma_Data_white = new ObservableCollection<Gamma_result>();
        public ObservableCollection<Gamma_result> Gamma_Data_blue = new ObservableCollection<Gamma_result>();
        public ObservableCollection<Gamma_result> Gamma_Data_green = new ObservableCollection<Gamma_result>();
        public ObservableCollection<Gamma_result> Gamma_Data_red = new ObservableCollection<Gamma_result>();
        public ObservableCollection<Gamma_distortion_item> Gamma_Distort_white = new ObservableCollection<Gamma_distortion_item>();
        public ObservableCollection<Gamma_distortion_item> Gamma_Distort_blue = new ObservableCollection<Gamma_distortion_item>();
        public ObservableCollection<Gamma_distortion_item> Gamma_Distort_green = new ObservableCollection<Gamma_distortion_item>();
        public ObservableCollection<Gamma_distortion_item> Gamma_Distort_red = new ObservableCollection<Gamma_distortion_item>();
        //public Gamma_Distortion? gamma_distort_white;
        //public Gamma_Distortion? gamma_distort_blue;
        //public Gamma_Distortion? gamma_distort_green;
        //public Gamma_Distortion? gamma_distort_red;
        private string CTF_Filename = "CTF.csv";
        private string Gamma_Filename = "Gamma.csv";

        public async void LoadCsvFileAsync()
        {
            try
            {
                string path = AppDomain.CurrentDomain.BaseDirectory + CTF_Filename;
                using (StreamReader sr = new StreamReader(path))
                {
                    while (sr.Peek() >= 0)
                    {
                        String? Line;
                        Line = await sr.ReadLineAsync();
                        if (Line != null)
                        {
                            if (Line.Contains("pixel"))
                            {
                                CTF_Data.Add(new CTF(Line.Split(',')[4], Line.Split(',')[6], Line.Split(',')[7], Line.Split(',')[8], Line.Split(',')[10], Line.Split(',')[11]));
                            }
                        }
                    }
                }
            }
            catch (FileNotFoundException ex)
            {
                Console.WriteLine(ex.Message);
            }

        }
        public async void LoadGammaCsvFileAsync()
        {
            try
            {
                string path = AppDomain.CurrentDomain.BaseDirectory + Gamma_Filename;
                using (StreamReader sr = new StreamReader(path))
                {
                    bool distort_start = false;
                    string item_name = "";
                    while (sr.Peek() >= 0)
                    {
                        String? Line;
                        Line = await sr.ReadLineAsync();
                        if (Line != null)
                        {
                            if (Line.Contains("White") && Line.Contains(".png"))
                            {
                                Gamma_Data_white.Add(new Gamma_result(Line.Split(',')[6], Line.Split(',')[7], Line.Split(',')[8], Line.Split(',')[9], Line.Split(',')[10], Line.Split(',')[11], Line.Split(',')[12]));
                            }
                            else if(Line.Contains("Blue") && Line.Contains(".png"))
                            {
                                Gamma_Data_blue.Add(new Gamma_result(Line.Split(',')[6], Line.Split(',')[7], Line.Split(',')[8], Line.Split(',')[9], Line.Split(',')[10], Line.Split(',')[11], Line.Split(',')[12]));
                            }
                            else if (Line.Contains("Green") && Line.Contains(".png"))
                            {
                                Gamma_Data_green.Add(new Gamma_result(Line.Split(',')[6], Line.Split(',')[7], Line.Split(',')[8], Line.Split(',')[9], Line.Split(',')[10], Line.Split(',')[11], Line.Split(',')[12]));
                            }
                            else if (Line.Contains("Red") && Line.Contains(".png"))
                            {
                                Gamma_Data_red.Add(new Gamma_result(Line.Split(',')[6], Line.Split(',')[7], Line.Split(',')[8], Line.Split(',')[9], Line.Split(',')[10], Line.Split(',')[11], Line.Split(',')[12]));
                            }
                            else if (Line.Contains("Distortion_up") && Line.Contains("Max_Distortion"))
                            {
                                distort_start = true;
                                item_name = Line;
                            }
                            else if (distort_start==true && Line.Contains("White"))
                            {
                                for(int i=1;i<item_name.Split(',').Length;i++)
                                    Gamma_Distort_white.Add(new Gamma_distortion_item(item_name.Split(',')[i], Line.Split(',')[i]));
                            }
                            else if (distort_start == true && Line.Contains("Blue"))
                            {
                                for (int i = 1; i < item_name.Split(',').Length; i++)
                                    Gamma_Distort_blue.Add(new Gamma_distortion_item(item_name.Split(',')[i], Line.Split(',')[i])); ;
                            }
                            else if (distort_start == true && Line.Contains("Green"))
                            {
                                for (int i = 1; i < item_name.Split(',').Length; i++)
                                    Gamma_Distort_green.Add(new Gamma_distortion_item(item_name.Split(',')[i], Line.Split(',')[i]));
                            }
                            else if (distort_start == true && Line.Contains("Red"))
                            {
                                for (int i = 1; i < item_name.Split(',').Length; i++)
                                    Gamma_Distort_red.Add(new Gamma_distortion_item(item_name.Split(',')[i], Line.Split(',')[i]));
                            }
                        }
                    }
                }
            }
            catch (FileNotFoundException ex)
            {
                Console.WriteLine(ex.Message);
            }

        }
        public void deleteCsv()
        {
            string Gamma_path= System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory + Gamma_Filename);
            string CTF_path = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory + CTF_Filename);
            if (File.Exists(CTF_path) == true)
            {
                File.Delete(CTF_path);
            }
            if (File.Exists(Gamma_path) == true)
            {
                File.Delete(Gamma_path);
            }
        }
    }
    public class Member : INotifyPropertyChanged
    {
        string? station;
        string? status;
        string? errormessage;


        public string Station
        {
            set
            {
                station = value;
                NotifyPropertyChanged();
            }
            get
            {
                return station;

            }
        }
        public string? Status
        {
            set
            {
                status = value;
                NotifyPropertyChanged();
            }
            get
            {
                return status;

            }
        }
        public string? ErrorMessage
        {
            set
            {
                errormessage = value;
                NotifyPropertyChanged();
            }
            get
            {
                return errormessage;
            }
        }
        public event PropertyChangedEventHandler? PropertyChanged;
        protected void NotifyPropertyChanged([CallerMemberName] string propertyName="")
        {
            if (PropertyChanged != null)
            { PropertyChanged(this, new PropertyChangedEventArgs(propertyName)); }
        }
    }
    public class CTF
    {
        public string color { get; set; }
        public string fov_h_deg { get; set; }
        public string fov_v_deg { get; set; }
        public string orientation { get; set; }
        public string spatial_period_lea_px { get; set; }
        public string ctf_percent { get; set; }
        public CTF(string color_in, string fov_h_deg_in, string fov_v_deg_in, string orientation_in, string spatial_period_lea_px_in, string ctf_percent_in)
        {
            color = color_in;
            fov_v_deg = fov_v_deg_in;
            fov_h_deg = fov_h_deg_in;
            orientation = orientation_in;
            spatial_period_lea_px = spatial_period_lea_px_in;
            ctf_percent = ctf_percent_in;
        }
    }
    public class Gamma_result
    {
        public string eyebox_x { get; set; }
        public string eyebox_y { get; set; }
        public string fov_v_deg { get; set; }
        public string fov_h_deg { get; set; }
        public string gray { get; set; }
        public string luminance { get; set; }
        public string gamma_result { get; set; }
        public Gamma_result(string eyebox_x_in,string eyebox_y_in,string fov_v_deg_in, string fov_h_deg_in,string gray_in,string luminance_in,string gamma_result_in) 
        {
            eyebox_x= eyebox_x_in;
            eyebox_y= eyebox_y_in;
            fov_h_deg= fov_h_deg_in;
            fov_v_deg= fov_v_deg_in;
            gray= gray_in;
            luminance= luminance_in;
            gamma_result= gamma_result_in;
        }
    }
    //public class Gamma_Distortion
    //{
    //    public string gamma_up { get; set; }
    //    public string gamma_left { get; set; }
    //    public string gamma_mid { get; set; }
    //    public string gamma_right { get; set; }
    //    public string gamma_down { get; set; }
    //    public string distort_up { get; set; }
    //    public string distort_left { get; set; }
    //    public string distort_right { get; set; }
    //    public string distort_down { get; set; }
    //    public string distort_max { get; set; }
    //    public string distort_ave { get; set; }

    //    public Gamma_Distortion(string gamma_up, string gamma_left , string gamma_mid , string gamma_right, string gamma_down, string distort_up , string distort_left, string distort_right, string distort_down, string distort_max, string distort_ave)
    //    {
    //        this.gamma_up = gamma_up;
    //        this.gamma_left = gamma_left;
    //        this.gamma_mid = gamma_mid;
    //        this.gamma_right = gamma_right;
    //        this.gamma_down = gamma_down;
    //        this.distort_up = distort_up;
    //        this.distort_left = distort_left;
    //        this.distort_right = distort_right;
    //        this.distort_down = distort_down;
    //        this.distort_max = distort_max;
    //        this.distort_ave = distort_ave;
    //    }
    //}
    public class Gamma_distortion_item
    {
        public string item { get; set;}
        public string result { get; set;}
        public Gamma_distortion_item(string item, string result)
        {
            this.item = item;
            this.result = result;
        }
    }
    public class UI
    {
        public ObservableCollection<Member> memberData;
        public string message_show = "";
        public bool error_bool;
        public UI()
        {
            memberData = new ObservableCollection<Member>();
            error_bool = false;
        }
        public async void Load_UI_StatusFileAsync()
        {
            try
            {
                string path = AppDomain.CurrentDomain.BaseDirectory + "UI_status.txt";
                using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
                using (StreamReader sr = new StreamReader(fs))
                {
                    while (sr.Peek() >= 0)
                    {
                        String? Line;
                        Line = await sr.ReadLineAsync();
                        if (Line != null)
                        {
                            if (Line.Split(',')[0] == "Brightness" || Line.Split(',')[0] == "CTF" || Line.Split(',')[0] == "Contrast" || Line.Split(',')[0] == "Uniformity" || Line.Split(',')[0] == "Capture")
                            {
                                string station = Line.Split(',')[0];
                                string status = Line.Split(',')[1];
                                string errorMessage = Line.Split(',')[2];

                                // 查找匹配的 Member 对象
                                Member existingMember = memberData.FirstOrDefault(member => member.Station == station);

                                // 如果找到匹配的 Member 对象，则更新属性值
                                if (existingMember != null)
                                {
                                    existingMember.Status = status;
                                    existingMember.ErrorMessage = errorMessage;
                                }
                                // 否则，向集合中添加新的 Member 对象
                                else
                                {
                                    Application.Current.Dispatcher.Invoke(() =>
                                    {
                                        memberData.Add(new Member() { Station = station, Status = status, ErrorMessage = errorMessage });
                                    });
                                }
                            }
                        }
                    }
                }
            }
            catch (FileNotFoundException ex)
            {
                Console.WriteLine(ex.Message);
                message_show = ex.Message;
            }
            catch (IOException ex)
            {
                Console.WriteLine(ex.Message);
                await Task.Delay(1000); // 等待1秒
                Load_UI_StatusFileAsync(); // 重新呼叫方法進行重試
            }
        }
        public void Load_UI_StatusFile()
        {
            string path = AppDomain.CurrentDomain.BaseDirectory + "UI_status.txt";

            bool retry = true;
            while (retry)
            {
                try
                {
                    using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
                    using (StreamReader sr = new StreamReader(fs))
                    {
                        while (sr.Peek() >= 0)
                        {
                            string Line = sr.ReadLine();
                            if (Line != null)
                            {
                                if (Line.Split(',')[0] == "Brightness" || Line.Split(',')[0] == "CTF" || Line.Split(',')[0] == "Contrast" || Line.Split(',')[0] == "Uniformity" || Line.Split(',')[0] == "Capture")
                                {
                                    string station = Line.Split(',')[0];
                                    string status = Line.Split(',')[1];
                                    string errorMessage = Line.Split(',')[2];

                                    // 查找匹配的 Member 对象
                                    Member existingMember = memberData.FirstOrDefault(member => member.Station == station);

                                    // 如果找到匹配的 Member 对象，则更新属性值
                                    if (existingMember != null)
                                    {
                                        existingMember.Status = status;
                                        existingMember.ErrorMessage = errorMessage;
                                    }
                                    // 否则，向集合中添加新的 Member 对象
                                    else
                                    {
                                        Application.Current.Dispatcher.Invoke(() =>
                                        {
                                            memberData.Add(new Member() { Station = station, Status = status, ErrorMessage = errorMessage });
                                        });
                                    }
                                }
                            }
                        }
                    }
                    retry = false; // 讀取成功，不需要重試
                }
                catch (FileNotFoundException ex)
                {
                    Console.WriteLine(ex.Message);
                    message_show = ex.Message;
                    retry = false; // 不需要重試，因為找不到檔案
                }
                catch (IOException ex)
                {
                    Console.WriteLine(ex.Message);
                    System.Threading.Thread.Sleep(1000); // 同步等待1秒
                }
            }
        }

    }
}
