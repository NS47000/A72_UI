   M S B A M L   `   `   `  ���� >  :A72, Version=1.0.0.0, Culture=neutral, PublicKeyToken=nullclr-namespace:A72A72      A72.MainWindow   W SPresentationCore, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35e aSystem.Windows.Controls.Ribbon, Version=6.0.2.0, Culture=neutral, PublicKeyToken=b77a5c561934e089R NWindowsBase, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35F 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     R NSystem.Xaml, Version=6.0.2.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     @d2http://schemas.microsoft.com/expression/blend/2008     Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     #localclr-namespace:A72     -     � Title$	  A72��$	��450��$	��800��.��� #��� �� � RowDefinitions �� $
��0.5* ��� �� �� �� �� �� �� �� �� ��  � ColumnDefinitions �� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* ��� $��1* �.+��� -    ��Name$ toolbar��
���   �$��1q�$��20��$��0,0,0,0q�$��1��#��� �� J� OpacityMask# � ��._�o� #��� ��.@�m� $��_file��$��1,1,1,1q�$
}�True�� J� SnapsToDevicePixels$
 True��.@�m� -   $��_Result Folder��m� -   $��_DUT Folder��m� $��_process��.@�m� -   $��	_CTF_data��m� -   $��	_Load_pos��m� -   $��	_Test_pos���� $��2��$	��SN:��$��0,0,0,0q�$��2��$��1��$��2��$��18(�$��Arial Black)�#��� ���� -    ��Name$ 
TextBox_SN��$��0,0,0,0q�$��2��$�� ��$��3��$��1��$��3���� $��6��$��Ope. ID:��$��0,0,0,0q�$��2��$��1��$��2��$��18(�$��Arial Black)�#��� /��� -	   $ TextBox_OperateID��$��0,0,0,0q�$��2��$�� ��$��8��$��1��$��3���� -
    ��Name$ imagebox��$��11��$��6��$
��Left=�$��0,0,0,0q�$��4��$��5��$	��Top=�$
��Fill=�$��/quanta_logo.png���� -    ��Name$ image_message��$��12��$��waiting....��$��0,0,0,0q�$��9��$��1��$��4��$��Center=�$��Center=����	#FFFDFDFD�� -   $��11��$��start��$��3��$��	44,41,0,0q�$��1��$��20(�$��42��$��3��$	��Top=����� 	 �� 
TargetType"	 ��.R��� $��2q�
���   �
 �� CornerRadius$
 15��#��z��._��� $��Center=�$��Center=�&   System.Windows.Controls.DataGrid  -     Name$ dataGrid��   AutoGenerateColumns$ False��$��2��$��8��$��4��$��6��$��10,20,10,10q�����    Columns 0  *System.Windows.Controls.DataGridTextColumn  ,  &System.Windows.Controls.DataGridColumn   Header$ statione��   Width5  /System.Windows.Controls.DataGridLengthConverter$ 80 1  +System.Windows.Controls.DataGridBoundColumn   Binding �� *	Station+  $ status��$ 50  �� *Status+5  /System.Windows.Controls.DataGridHyperlinkColumn  $ Detail��$ 1*  �� *ErrorMessage+�� $��2��$��8��$��4��$��6��$��result��$��0,0,0,0q��� -    ��Name$ toolbar_bottom��$��10��$��18��$��Bottom=�#��� ��.@��� .���� runing percentage�� .��� -   $��progressBar1��$	��200��$��20�� � Value$ 0Q� � Triggers :�  :� RoutedEvent$ ProgressBar.Loaded���� �� $��cost time： ���� -    ��Name$ time_calculate��$�� ��$��50���� �� $��
message：���� -   $ 	messenage��$�� ���� -   $��ComboBox�� �� Text$
 mode�� �� 
IsEditable$
 True�� �� 
IsReadOnly$
 True�� �� IsDropDownOpen$ False��$��0,1,0,2q�$��2��$��15��$��3��$��Center=�$��Center=�$��22(�.@��� $��Run All�� �� 
IsSelected$
 True���� $��
Brightness���� $	��CTF���� $��Contrast���� $��
Uniformity��