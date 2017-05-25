/*!
  \file
  \brief RS-232 シリアル通信 (Windows)

  \author Satofumi Kamimura

  $Id$
*/

#include <cstdio>
#include <windows.h>
#include <setupapi.h>
#include "Serial.h"
#include "Ring_buffer.hpp"

using namespace hrk;
using namespace std;

#if defined(VISUAL_CPP)
#define snprintf _snprintf
#endif


namespace
{
    string find_port_driver(HKEY hkey, LPCSTR subkey, const string& port_name)
    {
        HKEY next_hkey;
        if (RegOpenKeyExA(hkey, subkey,
                          0, KEY_READ, &next_hkey) != ERROR_SUCCESS) {
            return "";
        }

        // COM と一致するものがあるか調べる
        enum { MaxLength = 1024 };
        CHAR device[MaxLength + 1];
        char name[MaxLength + 1];

        DWORD ret = ERROR_SUCCESS;
        for (int i = 0; ret == ERROR_SUCCESS; ++i) {
            DWORD dl = MaxLength;
            DWORD nl = MaxLength;
            ret = RegEnumValueA(hkey, i, device, &dl,
                                NULL, NULL, (BYTE*)name, &nl);
            if (ret != ERROR_SUCCESS) {
                break;
            }

            char* first_p = strrchr(name, '(');
            if (!first_p) {
                continue;
            }
            char* last_p = strrchr(name, ')');
            if (!last_p) {
                continue;
            }
            if ((last_p - first_p - 1) != static_cast<int>(port_name.size())) {
                continue;
            }

            if (!strncmp(first_p + 1, port_name.data(), port_name.size())) {
                RegCloseKey(next_hkey);
                return string(name).substr(0, first_p - name - 1);
            }
        }

        // find_value が存在しなければ、再帰探索を行う
        char next_subkey[MaxLength];
        FILETIME filetime;

        ret = ERROR_SUCCESS;
        for (int i = 0; ret == ERROR_SUCCESS; ++i) {
            DWORD dl = MaxLength, nl = MaxLength;
            ret = RegEnumKeyExA(next_hkey, i, next_subkey,
                                &dl, NULL, NULL, &nl, &filetime);
            if (ret != ERROR_SUCCESS) {
                break;
            }

            string found_name =
                find_port_driver(next_hkey, next_subkey, port_name);
            if (!found_name.empty()) {
                RegCloseKey(next_hkey);
                return found_name;
            }
        }

        RegCloseKey(next_hkey);
        return "";
    }
}


struct Serial::pImpl
{
    string error_message_;
    HANDLE com_;
    int current_timeout_;
    Ring_buffer<char> ring_buffer_;


    pImpl(void)
        : error_message_("no error."), com_(INVALID_HANDLE_VALUE),
          current_timeout_(0)
    {
    }


    bool open(const char* device_name, long baudrate)
    {
        close();

        // COM ポートを開く
        enum { NameLength = 11 };
        char adjusted_device[NameLength];
        snprintf(adjusted_device, NameLength, "\\\\.\\%s", device_name);
        com_ = CreateFileA(adjusted_device, GENERIC_READ | GENERIC_WRITE,
                           0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (com_ == INVALID_HANDLE_VALUE) {
            error_message_ = string(device_name) + ": open failed.";
            return false;
        }

        // 通信サイズの更新
        SetupComm(com_, 4096 * 8, 4096);

        // タイムアウトの設定
        set_timeout(current_timeout_);

        // ボーレートの変更
        return set_baudrate(baudrate);
    }


    void close(void)
    {
        if (is_open()) {
            CloseHandle(com_);
            com_ = INVALID_HANDLE_VALUE;
        }
    }


    bool is_open(void) const
    {
        return (com_ == INVALID_HANDLE_VALUE) ? false : true;
    }


    bool set_baudrate(long baudrate)
    {
        typedef struct
        {
            long baudrate;
            long value;
        } baudrate_table_t;

        baudrate_table_t table[] = {
            { 4800, CBR_4800 },
            { 9600, CBR_9600 },
            { 19200, CBR_19200 },
            { 38400, CBR_38400 },
            { 57600, CBR_57600 },
            { 115200, CBR_115200 },
        };

        enum { Invalid_value = -1 };
        long baudrate_value = Invalid_value;
        size_t table_size = sizeof(table) / sizeof(table[0]);
        for (size_t i = 0; i < table_size; ++i) {
            if (table[i].baudrate == baudrate) {
                baudrate_value = table[i].value;
                break;
            }
        }
        if (baudrate_value == Invalid_value) {
            baudrate_value = baudrate;
        }

        DCB dcb;
        GetCommState(com_, &dcb);
        dcb.BaudRate = baudrate_value;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.fParity = FALSE;
        dcb.StopBits = ONESTOPBIT;
        SetCommState(com_, &dcb);

        clear();

        return true;
    }


    void clear(void)
    {
        // 実装しない
    }


    void set_timeout(int timeout)
    {
        COMMTIMEOUTS timeouts;
        GetCommTimeouts(com_, &timeouts);

        timeouts.ReadIntervalTimeout = (timeout == 0) ? MAXDWORD : 0;
        timeouts.ReadTotalTimeoutConstant = timeout;
        timeouts.ReadTotalTimeoutMultiplier = 0;

        SetCommTimeouts(com_, &timeouts);
    }


    int read(char* data, size_t max_data_size, int timeout)
    {
        if (!is_open()) {
            error_message_ = "not opened.";
            return -1;
        }

        if (max_data_size <= 0) {
            return 0;
        }

        int filled_size = 0;
        int buffer_size = ring_buffer_.size();
        int read_size = max_data_size - filled_size;
        if (buffer_size < read_size) {
            // リングバッファ内のデータで足りなければ、データを読み足す
            enum { Buffer_size = 4096 };
            char buffer[Buffer_size];
            int n = internal_receive(buffer, Buffer_size, 0);
            if (n > 0) {
                buffer_size += ring_buffer_.push(buffer, n);
            }
        }

        // リングバッファ内のデータを返す
        read_size = min(read_size, buffer_size);
        if (read_size > 0) {
            filled_size += ring_buffer_.pop(&data[filled_size], read_size);
        }

        // データをタイムアウト付きで読み出す
        filled_size += internal_receive(&data[filled_size],
                                        max_data_size - filled_size, timeout);
        return filled_size;
    }


    int internal_receive(char data[], int data_size_max, int timeout)
    {
        if (data_size_max <= 0) {
            return 0;
        }

        if (timeout != current_timeout_) {
            set_timeout(timeout);
            current_timeout_ = timeout;
        }

        int filled_size = 0;
        while (filled_size < data_size_max) {
            int require_n;
            DWORD read_n;

            require_n = data_size_max - filled_size;
            ReadFile(com_, &data[filled_size], (DWORD)require_n, &read_n, NULL);

            if (read_n <= 0) {
                // 読み出しエラー。現在までの受信内容で戻る
                break;
            }
            filled_size += read_n;
        }
        return filled_size;
    }
};


Serial::Serial(void) : pimpl(new pImpl)
{
}


Serial::~Serial(void)
{
    close();
}


std::vector<std::string> Serial::find_ports(void)
{
    // デバイスマネージャの一覧から COM デバイスを探す

    //4D36E978-E325-11CE-BFC1-08002BE10318
    GUID GUID_DEVINTERFACE_COM_DEVICE = {
        0x4D36E978L, 0xE325, 0x11CE,
        {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 }
    };

    HDEVINFO hdi = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COM_DEVICE, 0, 0,
                                       DIGCF_PRESENT | DIGCF_INTERFACEDEVICE);

    vector<string> found_ports;
    if (hdi == INVALID_HANDLE_VALUE) {
        return found_ports;
    }

    SP_DEVINFO_DATA sDevInfo;
    sDevInfo.cbSize = sizeof(SP_DEVINFO_DATA);
    for (int i = 0; SetupDiEnumDeviceInfo(hdi, i, &sDevInfo); ++i){
        enum {
            BufferSize = 256,
            ComNameLengthMax = 7,
        };
        char buffer[BufferSize + 1];
        DWORD dwRegType;
        DWORD dwSize;

        // フレンドリーネームを取得して COM 番号を取り出す
        SetupDiGetDeviceRegistryPropertyA(hdi, &sDevInfo, SPDRP_FRIENDLYNAME,
                                          &dwRegType, (BYTE*)buffer, BufferSize,
                                          &dwSize);
        int n = (int)strlen(buffer);
        if (n < ComNameLengthMax) {
            // COM 名が短過ぎる場合、処理しない
            continue;
        }

        // (COMx) の最後の括弧の位置に '\0' を代入する
        char* p = strrchr(buffer, ')');
        if (p) {
            *p = '\0';
        }

        // COM と番号までの文字列を抜き出す
        p = strstr(&buffer[n - ComNameLengthMax], "COM");
        if (! p) {
            continue;
        }

        found_ports.push_back(p);
    }
    SetupDiDestroyDeviceInfoList(hdi);

    return found_ports;
}


std::string Serial::port_driver_name(const string& port_name)
{
    return find_port_driver(HKEY_LOCAL_MACHINE,
                            "SYSTEM\\CurrentControlSet\\Enum\\USB", port_name);
}


const char* Serial::what(void) const
{
    return pimpl->error_message_.c_str();
}


bool Serial::open(const std::string& device_name, long baudrate)
{
    return pimpl->open(device_name.c_str(), baudrate);
}


bool Serial::change_baudrate(long baudrate)
{
    return pimpl->set_baudrate(baudrate);
}


bool Serial::is_open(void) const
{
    return pimpl->is_open();
}


void Serial::close(void)
{
    pimpl->close();
}


int Serial::write(const char* data, size_t data_size)
{
    if (!is_open()) {
        pimpl->error_message_ = "not opened.";
        return -1;
    }

    if (pimpl->com_ == INVALID_HANDLE_VALUE) {
        return -1;
    }

    DWORD n;
    WriteFile(pimpl->com_, data, (DWORD)data_size, &n, NULL);
    return n;

}


int Serial::read(char* data, size_t max_data_size, int timeout)
{
    return pimpl->read(data, max_data_size, timeout);
}


void Serial::ungetc(int ch)
{
    pimpl->ring_buffer_.ungetc(ch);
}
