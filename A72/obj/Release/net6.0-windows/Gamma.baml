   M S B A M L   `   `   `  ���� >  :A72, Version=1.0.0.0, Culture=neutral, PublicKeyToken=nullclr-namespace:A72A72      	A72.Gamma   W SPresentationCore, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35e aSystem.Windows.Controls.Ribbon, Version=6.0.2.0, Culture=neutral, PublicKeyToken=b77a5c561934e089R NWindowsBase, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35F 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     R NSystem.Xaml, Version=6.0.2.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     @d2http://schemas.microsoft.com/expression/blend/2008     Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     #localclr-namespace:A72       � Title$  Gamma��$	��550��$
��1600��$��20(�.���  � RowDefinitions �� �� �� �� �� �� �� �� �� �� ��  � ColumnDefinitions �� �� �� �� �� �� �� �� �� �� �� �� �� �� �� �� .+��� $��1��$��1��$��9��$��14��.@��� $��White��$��20(�.���  �� $
��1.4* ��� �� $
��0.5* �.+��� -    ��Name$ Gamma_result_list_white��$��0��$��17��$��2��$��22������  �� View ��  �� AllowsColumnReorder$
 true�� �� ColumnHeaderToolTip$ Employee Information��.P��� $��EyeBox_x�� �� Width$ 90�� �� DisplayMemberBinding �� 	 �� Path$	 eyebox_x��� $��EyeBox_y��$ 90�� �� $	 eyebox_y��� $��	Fov_h_deg��$	 100�� �� $	 	fov_h_deg��� $��	Fov_v_deg��$	 100�� �� $	 	fov_v_deg��� $
��Gray��$ 60�� �� $
	 gray��� $��	Luminance��$	 120�� �� $	 	luminance��� $��Gamma��$	 100�� �� $	 gamma_result��� -   
 ��Name$
 imagebox_white��$��1��$
��Fill=�&   System.Windows.Controls.DataGrid  -     Name$ Gamma_distort_data_white��   AutoGenerateColumns$ False��$��2������    Columns 0  *System.Windows.Controls.DataGridTextColumn  ,  &System.Windows.Controls.DataGridColumn   Header$
 Item��   Width5  /System.Windows.Controls.DataGridLengthConverter$ 2* 1  +System.Windows.Controls.DataGridBoundColumn   Binding �� *item+  $ Result��$ 1*  �� *result+�� $
��Blue��$��20(�.���  �� $
��1.4* ��� �� $
��0.5* �.+��� -   $ Gamma_result_list_blue��$��0��$��17��$��2��$��22������  �� $
 true��$ Employee Information��.P��� $��EyeBox_x��$ 90�� �� $	 eyebox_x��� $��EyeBox_y��$ 90�� �� $	 eyebox_y��� $��	Fov_h_deg��$	 100�� �� $	 	fov_h_deg��� $��	Fov_v_deg��$	 100�� �� $	 	fov_v_deg��� $
��Gray��$ 60�� �� $
	 gray��� $��	Luminance��$	 120�� �� $	 	luminance��� $��Gamma��$	 100�� �� $	 gamma_result��� -   $
 imagebox_blue��$��1��$
��Fill=�  -   $ Gamma_distort_data_blue��$ False��$��2������    $
 Item��$ 2*  �� *item+  $ Result��$ 1*  �� *result+�� $��Green��$��20(�.���  �� $
��1.4* ��� �� $
��0.5* �.+��� -   $ Gamma_result_list_green��$��0��$��17��$��2��$��22������  �� $
 true��$ Employee Information��.P��� $��EyeBox_x��$ 90�� �� $	 eyebox_x��� $��EyeBox_y��$ 90�� �� $	 eyebox_y��� $��	Fov_h_deg��$	 100�� �� $	 	fov_h_deg��� $��	Fov_v_deg��$	 100�� �� $	 	fov_v_deg��� $
��Gray��$ 60�� �� $
	 gray��� $��	Luminance��$	 120�� �� $	 	luminance��� $��Gamma��$	 100�� �� $	 gamma_result��� -   $
 imagebox_green��$��1��$
��Fill=�  -	   $ Gamma_distort_data_green��$ False��$��2������    $
 Item��$ 2*  �� *item+  $ Result��$ 1*  �� *result+�� $	��Red��$��20(�.���  �� $
��1.4* ��� �� $
��0.5* �.+��� -
   $ Gamma_result_list_red��$��0��$��17��$��2��$��22������  �� $
 true��$ Employee Information��.P��� $��EyeBox_x��$ 90�� �� $	 eyebox_x��� $��EyeBox_y��$ 90�� �� $	 eyebox_y��� $��	Fov_h_deg��$	 100�� �� $	 	fov_h_deg��� $��	Fov_v_deg��$	 100�� �� $	 	fov_v_deg��� $
��Gray��$ 60�� �� $
	 gray��� $��	Luminance��$	 120�� �� $	 	luminance��� $��Gamma��$	 100�� �� $	 gamma_result��� -   $
 imagebox_red��$��1��$
��Fill=�  -   $ Gamma_distort_data_red��$ False��$��2������    $
 Item��$ 2*  �� *item+  $ Result��$ 1*  �� *result+