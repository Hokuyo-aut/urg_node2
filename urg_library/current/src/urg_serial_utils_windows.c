/*!
  \file
  \~japanese 
  \brief シリアル用の補助関数
  \~english
  \brief Serial communication auxiliary functions for Windows
  \author Satofumi KAMIMURA

  $Id$

  \~japanese 
  \todo 変数名を '_' 区切りの形式に変更する
  \todo C90 相当で動作するように調整する。ただし、"//" コメントは使う
  \~english
  \todo Change variable names to use the "_" convention
  \todo Fix the code to be C90 compliant, but keeping the "//" comments
*/

#include "urg_serial_utils.h"
#include "urg_detect_os.h"
#include <windows.h>
#include <setupapi.h>
#include <string.h>
#include <stdio.h>


#if defined(URG_MSC)
#define snprintf _snprintf
#endif


enum {
    MAX_PORTS = 16,
    DEVICE_NAME_SIZE = 7,
};


static char found_ports[MAX_PORTS][DEVICE_NAME_SIZE];
static int is_urg_ports[MAX_PORTS];
static int found_ports_size = 0;

static char *search_driver_names[] = {
    "URG Series USB Device Driver",
    "URG-X002 USB Device Driver",
};


static void swap_item(int from_index, int to_index)
{
    char buffer[DEVICE_NAME_SIZE];
    int is_urg_port;

    if (from_index == to_index) {
        return;
    }

    strncpy(buffer, found_ports[to_index], DEVICE_NAME_SIZE);
    strncpy(found_ports[to_index], found_ports[from_index], DEVICE_NAME_SIZE);
    strncpy(found_ports[from_index], buffer, DEVICE_NAME_SIZE);

    is_urg_port = is_urg_ports[to_index];
    is_urg_ports[to_index] = is_urg_ports[from_index];
    is_urg_ports[from_index] = is_urg_port;
}


static void sort_ports(void)
{
    int last_index = 0;
    int i;

    for (i = 0; i < found_ports_size; ++i) {
        if ((is_urg_ports[i] == 1) && (last_index < i)) {
            swap_item(i, last_index);
            last_index = i + 1;
        }
    }
}


int urg_serial_find_port(void)
{
    // \~japanese デバイスマネージャの一覧から COM デバイスを探す
    // \~english Searchs for COM devices from the list of device manager

    //4D36E978-E325-11CE-BFC1-08002BE10318
    GUID GUID_DEVINTERFACE_COM_DEVICE = {
        0x4D36E978L, 0xE325, 0x11CE,
        {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 }
    };

    HDEVINFO hdi;
    SP_DEVINFO_DATA sDevInfo;
    int i;

    found_ports_size = 0;
    hdi = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COM_DEVICE, 0, 0,
                              DIGCF_PRESENT | DIGCF_INTERFACEDEVICE);
    if (hdi == INVALID_HANDLE_VALUE) {
        return 0;
    }

    sDevInfo.cbSize = sizeof(SP_DEVINFO_DATA);
    for (i = 0; SetupDiEnumDeviceInfo(hdi, i, &sDevInfo); ++i){

        enum {
            BufferSize = 256,
            ComNameLengthMax = 7,
        };
        char buffer[BufferSize + 1];
        DWORD dwRegType;
        DWORD dwSize;
        int is_urg_port;
        char *p;
        int n;
        int j;

        // \~japanese フレンドリーネームを取得して COM 番号を取り出す
        // \~english Gets the Friendly name property and gets the COM port number
        SetupDiGetDeviceRegistryPropertyA(hdi, &sDevInfo, SPDRP_FRIENDLYNAME,
                                          &dwRegType, (BYTE*)buffer, BufferSize,
                                          &dwSize);
        n = (int)strlen(buffer);
        if (n < ComNameLengthMax) {
            // \~japanese COM 名が短過ぎた場合、処理しない
            // \~japanese 問題がある場合は、修正する
            // \~english If the COM name is too small, ignore it
            // \~english (fix this in case of problems)
            continue;
        }

        // \~japanese (COMx) の最後の括弧の位置に '\0' を代入する
        // \~english Adds '\0' at the end of the string
        p = strrchr(buffer, ')');
        if (p) {
            *p = '\0';
        }

        // \~japanese COM と番号までの文字列を抜き出す
        // \~english Split name into "COM" and the number
        p = strstr(&buffer[n - ComNameLengthMax], "COM");
        if (! p) {
            continue;
        }

        snprintf(found_ports[found_ports_size], DEVICE_NAME_SIZE, "%s", p);

        // \~japanese デバイス名を取得し、URG ポートかの判定に用いる
        // \~english Get the device names and detect the URG port
        SetupDiGetDeviceRegistryPropertyA(hdi, &sDevInfo, SPDRP_DEVICEDESC,
                                          &dwRegType, (BYTE*)buffer, BufferSize,
                                          &dwSize);
        is_urg_port = 0;
        n = sizeof(search_driver_names) / sizeof(search_driver_names[0]);
        for (j = 0; j < n; ++j) {
            if (! strcmp(search_driver_names[j], buffer)) {
                is_urg_port = 1;
                break;
            }
        }
        is_urg_ports[found_ports_size] = is_urg_port;
        ++found_ports_size;
    }
    SetupDiDestroyDeviceInfoList(hdi);

    // \~japanese is_urg_port の要素が先頭に来るようにソートする
    // \~english sort the elements in is_urg_port array
    sort_ports();

    return found_ports_size;
}


const char *urg_serial_port_name(int index)
{
    if ((index < 0) || (index >= found_ports_size)) {
        return "";
    } else {
        return found_ports[index];
    }
}


int urg_serial_is_urg_port(int index)
{
    if ((index < 0) || (index >= found_ports_size)) {
        return -1;
    } else {
        return is_urg_ports[index];
    }
}
