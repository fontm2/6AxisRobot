   M S B A M L   `   `   `  ����W  SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089   ColumnHeaderGripperStyle  �� 
TargetType �� Property �� Value �� 
TargetType �� Padding  CustomDataGridHeader=   7System.Windows.Controls.Primitives.DataGridColumnHeader'  !System.Windows.VisualStateManager   VisualStateGroups%  System.Windows.VisualStateGroup   States   System.Windows.VisualState   
Storyboard �� 
TargetName	 �� TargetProperty8   2System.Windows.Media.Animation.EasingColorKeyFrame
 �� KeyTime �� Value ��Name �� 
StartPoint �� EndPoint � Offset � Color J� SnapsToDevicePixels  CustomDataGridRow)  #System.Windows.Controls.DataGridRow �� Color?  9System.Windows.Controls.Primitives.SelectiveScrollingGrid � ColumnDefinitions � RowDefinitions?  9System.Windows.Controls.Primitives.DataGridCellsPresenter   
ItemsPanel*  $System.Windows.Controls.DataGridCell  CustomDatagrid&	   System.Windows.Controls.DataGrid �� Opacity 	  SelectAllCommand �� RelativeSource �� AncestorType �� Path �� TypeInTargetAssembly �� 
ResourceId! 	  HeadersVisibilityConverter �� 	Converter7
  1System.Windows.Controls.DataGridHeadersVisibility
 
  All  �� ConverterParameterG  ASystem.Windows.Controls.Primitives.DataGridColumnHeadersPresenter! 
  Column" ��Name# �� CanContentScroll$ �� Orientation% �� ScrollableHeight& � Maximum' �� ViewportHeight( �� ViewportSize*) �� #ComputedVerticalScrollBarVisibility* � Value+ �� Mode, �� ScrollableWidth- �� ViewportWidth,. �� %ComputedHorizontalScrollBarVisibility/ �� Triggers0 P� Property1 P� Value�� 5      D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     :x,http://schemas.microsoft.com/winfx/2006/xaml      5      %  &	        &	     &	 {    '  �    &	 �    �� 5      "  n�6-   .�5   
   ��  � ��$ 5��6"   �� 5   
    � ��
 ���� 6'   �� 5   
    �@�Cursor$ SizeWE��6#   �� 5	   
    � �� 5
      �� 5      " n�6"   .R�5      �� #��z��6   # z��6G   5      5      5      �� 5      "    6)   .�5   
   ��  �@�OverridesDefaultStyle . 62   �� 5   
    � ��$ Center=�67   �� 5   
    � ��$ Center=�65   �� 5   
    � ��
 �����6'   �� 5   
    � �� 5      �� 5      "   6"   .R�5      �  5        5      CommonStates��. 5   "     
Normal��6.     5   "   	MouseOver��. 5    &   �� .��5!   *   �� $ columnHeaderBorder��6G   $N	 H(Panel.Background).(GradientBrush.GradientStops)[1].(GradientStop.Color)�6r   .g�5"   .     $
 0��6B   $ White��6N   5#   +   5$   '   5%   #   5&      .+�5(      �� $ columnHeaderBorder��6!   $��1q�6=   $ 1q�6Q   ��5)      �� 5*   "   $ 0.5,0&�66   $ 0.5,1&�6I   .��5+   &   � $ 0Q�63   $ White��6>   � 5,   &   $ 1Q�63   $ White��6>   5-   #   ��5/      �� 50   "   $	 0,0&�66   $	 0,1&�6G   ��52   *   � � 53   .   $ Black��6;   $	 0.0Q�6I   � 54   .   $
 Gray��6;   $	 1.0Q�6H   55   +   57   #   ._�59      �� #��z��6/   #��z��6r   # z 6�   5:      5;      5<      5>      5?      �� 5A      "   6&   .�5B   
   ��  � ��
 �����6'   �� 5C   
    �@=�ErrorTemplate 65   Z� �� 5D   
    � �� 5E      �� 5F      "  6"   .R�5G      �� $ 
DGR_Border��6   
�������$��1,0.5,1,0.5q�6E    . ��5H      �� 5I      $ Transparent��6.   ._�5K         5L      �� 5M   "   $
��Auto �63   �� 5N   "   $��* �63    5P      �� 5Q   "   $��* �60   �� 5R   "   $
��Auto �60   .+�5T        $��1��65   #��z 6E   # z 6o   5V      5W      5X      5Z      5[      �� 5]      "   6   .�5^   
   ��  � �� 5_      �� 5`      "  6"   .R�5a      �� $ border��6   
������ 
������ $��1q�6`    . ._�5b      �� # z 6+   5c      5d      5f      5g      �� 5k      "  	 6#   .�5l   
   ��   �@	 RowDetailsVisibilityMode$ VisibleWhenSelected=�65   �� 5m   
    � �� . 6:   �� 5n   
    �@��PanningMode$
 Both=�65   �� 5o   
    �@��IsFlicksEnabled .  63   �� 5p   
    � ��$ 5q�6#   �� 5q   
    � ��
 �����6(   �� 5r   
     �@	 AlternatingRowBackground
 �����65   �� 5s   
     �@	 HorizontalGridLinesBrush
 �����65   �� 5t   
    �@	 VerticalGridLinesBrush
 �����63   �� 5u   
    � ��$
 Auto=�6G   �� 5v   
    � ��$
 Auto=�6E   �� 5w   
    � �� 5x      �� 5y      " 	 6"   .R�5z      ]� .+�5{      �� $��Border��66   $	��Top=�6!   $��2q�6D   ��5|      �� 5}   "   $ White��62   $	 0.3Q�6@   ��5      �� 5�   "   $ White��62   ._�5�      �� $��DG_ScrollViewer��6=   |�.  ��5�   "   �� 5�   &   " ��66   .R�5�   *   �  5�   .   �� 5�   2   $
��Auto �6@   �� 5�   2   $��* �6@   �� 5�   2   $
��Auto �6@    5�   .   �� 5�   2   $
��Auto �6C   �� 5�   2   $��* �6C   �� 5�   2   $
��Auto �6C   .+�5�   .   �� #��Z 65   |�.  ��6d   ��  �� " 	 $  CellsPanelHorizontalOffset���6�   C� *�� " 	 $" DataGridSelectAllButtonStyle��+y�6i  ��  �� " 	 $ HeadersVisibility�# Z #  Z   5�   .   $!��PART_ColumnHeadersPresenter��6i   $��1��6M   $��30��6]   y�6�   ��  �� " 	 $ HeadersVisibility�# Z #  Z! �� 5�   .   $!" PART_ScrollContentPresenter��6E   $��1��6j   $��2��6w   ## z��6�   �� 5�   .   $��PART_VerticalScrollBar��6f   $��0��68   $��2��6E   $��2��6V   $$ Vertical=�6�   #& z% 6�   #( z' 6�   #y�z) 6\  * 6�   �� $ VerticalOffset� �� *TemplatedParent+$+ OneWay=�6\  � 5�   .   $��2��63   $��1��6@    5�   2   �� 5�   6   ��6G   ��  �� " 	 $. (NonFrozenColumnsViewportHorizontalOffset��� 5�   6   $��* �6G   .+�5�   .   �� $��PART_HorizontalScrollBar��6H   $��1��68   $$ 
Horizontal=�6h   #& z, 6�   #( z- 6�   #y�z. 6B  * 6�   �� $ HorizontalOffset� �� *TemplatedParent+$+ OneWay=�6B  5�   /   5�   +   5�   '   .��5�   "   �� # z 61   5�      5�      5�      5�      5�      / 5�   
   P� 5�      0 �@��
IsGrouping1 . . �5�      ��  � �� .  6B   5�      5�      5�      