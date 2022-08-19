@echo off

if not exist vsvars32.bat goto end

CALL vsvars32.bat

REM Compile URG library

cl.exe -c -MD -I../include/c ../src/urg_sensor.c
cl.exe -c -MD -I../include/c ../src/urg_utils.c
cl.exe -c -MD -I../include/c ../src/urg_connection.c
cl.exe -c -MD -I../include/c ../src/urg_serial.c
cl.exe -c -MD -I../include/c ../src/urg_serial_utils.c
cl.exe -c -MD -I../include/c ../src/urg_tcpclient.c
cl.exe -c -MD -I../include/c ../src/urg_ring_buffer.c
cl.exe -c -MD -I../include/c ../src/urg_debug.c
lib.exe /OUT:urg.lib urg_sensor.obj urg_utils.obj urg_connection.obj urg_serial.obj urg_serial_utils.obj urg_tcpclient.obj urg_ring_buffer.obj urg_debug.obj
cl.exe /EHsc -c -MD -I../include/cpp ../src/ticks.cpp
cl.exe /EHsc -c -MD -I../include/cpp -I../include/c ../src/Urg_driver.cpp
lib.exe /OUT:urg_cpp.lib ticks.obj Urg_driver.obj urg.lib

REM Compile sample utility

cl.exe -c -MD -I../include/c ../samples/c/open_urg_sensor.c
cl.exe /EHsc -c -MD -I../include/cpp ../samples/cpp/Connection_information.cpp

REM Compile samples linking with ws2_32.lib setupapi.lib with /MD option.

cl.exe /MD -I../include/c ../samples/c/sensor_parameter.c open_urg_sensor.obj ws2_32.lib setupapi.lib urg.lib

cl.exe /MD -I../include/c ../samples/c/calculate_xy.c open_urg_sensor.obj ws2_32.lib setupapi.lib urg.lib

cl.exe /MD -I../include/c ../samples/c/get_multiecho_intensity.c open_urg_sensor.obj ws2_32.lib setupapi.lib urg.lib

cl.exe /MD -I../include/c ../samples/c/find_port.c open_urg_sensor.obj ws2_32.lib setupapi.lib urg.lib

cl.exe /MD -I../include/c ../samples/c/get_distance.c open_urg_sensor.obj ws2_32.lib setupapi.lib urg.lib

cl.exe /MD -I../include/c ../samples/c/get_distance_intensity.c open_urg_sensor.obj ws2_32.lib setupapi.lib urg.lib

cl.exe /MD -I../include/c ../samples/c/sync_time_stamp.c open_urg_sensor.obj ws2_32.lib setupapi.lib urg.lib

cl.exe /MD -I../include/c ../samples/c/get_multiecho.c open_urg_sensor.obj ws2_32.lib setupapi.lib urg.lib

cl.exe /MD /EHsc /Feget_distance_cpp.exe -I../include/cpp ../samples/cpp/get_distance.cpp Connection_information.obj urg_cpp.lib ws2_32.lib setupapi.lib

cl.exe /MD /EHsc /Feget_distance_intensity_cpp.exe -I../include/cpp ../samples/cpp/get_distance_intensity.cpp Connection_information.obj urg_cpp.lib ws2_32.lib setupapi.lib

cl.exe /MD /EHsc /Feget_multiecho_cpp.exe -I../include/cpp ../samples/cpp/get_multiecho.cpp Connection_information.obj urg_cpp.lib ws2_32.lib setupapi.lib

cl.exe /MD /EHsc /Feget_multiecho_intensity_cpp.exe -I../include/cpp ../samples/cpp/get_multiecho_intensity.cpp Connection_information.obj urg_cpp.lib ws2_32.lib setupapi.lib

cl.exe /MD /EHsc /Fesensor_parameter_cpp.exe -I../include/cpp ../samples/cpp/sensor_parameter.cpp Connection_information.obj urg_cpp.lib ws2_32.lib setupapi.lib

cl.exe /MD /EHsc /Fesync_time_stamp_cpp.exe -I../include/cpp ../samples/cpp/sync_time_stamp.cpp Connection_information.obj urg_cpp.lib ws2_32.lib setupapi.lib

echo ビルドが完了しました。終了します。
echo Built completed successfully.
set /p TMP=""
exit /b

:end
echo vsvars32.bat が見つかりません。終了します。
echo Error: vsvars32.bat could not be found
set /p TMP=""
exit /b
