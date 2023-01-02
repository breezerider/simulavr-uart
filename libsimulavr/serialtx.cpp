/*
 ****************************************************************************
 *
 * simulavr - A simulator for the Atmel AVR family of microcontrollers.
 * Copyright (C) 2001, 2002, 2003, 2004   Klaus Rudolph
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 ****************************************************************************
 *
 *  $Id$
 */

#include <cstring>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>

#include "ui/serialtx.h"
#include "systemclock.h"
#include "string2.h"
#include "avrerror.h"



SerialTxBuffered::SerialTxBuffered()
{
    allPins["tx"] = &tx;
    Reset();
}

void SerialTxBuffered::Reset()
{
    txState=TX_DISABLED;
    receiveInHex=false;
    baudrate=115200;
    maxBitCnt=8;
    tx='H'; 
}

Pin* SerialTxBuffered::GetPin(const char* name)
{
    return allPins[name];
}

int SerialTxBuffered::Step(
        bool &trueHwStep, SystemClockOffset *timeToNextStepIn_ns)
{
    switch (txState) {
        case TX_SEND_STARTBIT:
            data=*(inputBuffer.begin());
            inputBuffer.erase(inputBuffer.begin());
            tx='L';
            bitCnt=0;
            //all we measures are in ns !;
            *timeToNextStepIn_ns=(SystemClockOffset)1e9/baudrate;
            txState=TX_SEND_DATABIT;
            break;

        case TX_SEND_DATABIT:
            if (( data >> bitCnt ) & 0x01)  {
                tx='H';
            } else {
                tx='L';
            }
            //all we measures are in ns !;
            *timeToNextStepIn_ns=(SystemClockOffset)1e9/baudrate;
            bitCnt++;
            if(bitCnt>=maxBitCnt) txState=TX_SEND_STOPBIT;
            break;

        case TX_SEND_STOPBIT:
            tx='H';
            txState=TX_STOPPING;
            //all we measures are in ns !;
            *timeToNextStepIn_ns=(SystemClockOffset)1e9/baudrate;
            break;

        case TX_STOPPING:
            if (inputBuffer.size()>0) { //another char to send!
                txState=TX_SEND_STARTBIT;
                *timeToNextStepIn_ns=0;//(SystemClockOffset)1e9/baudrate;
            } else {
                txState=TX_DISABLED;
                *timeToNextStepIn_ns=-1; //last step, do not continue
            }

            break;

        default:
            avr_error("Illegal state in SerialTx");
    }


    return 0;

}

void SerialTxBuffered::Send(unsigned char data)
{
    inputBuffer.push_back(data); //write new char to input buffer

    //std::cerr << "TX: " << std::hex << data << " ";
    //if we not active, activate tx machine now
    if (txState==TX_DISABLED) {
        txState=TX_SEND_STARTBIT;
        SystemClock::Instance().Add(this);
    }
}

void SerialTxBuffered::SetBaudRate(SystemClockOffset baud){
    baudrate = baud;
}

void SerialTxBuffered::SetHexInput(bool newValue){
    receiveInHex = newValue;
}


// ===========================================================================
// ===========================================================================
// ===========================================================================



SerialTxFile::SerialTxFile(const char *filename) {
    if (std::strcmp(filename, "-") == 0)
#ifndef WIN32
        fd = fileno(stdin);
#else
        hFile = GetStdHandle(STD_INPUT_HANDLE);
#endif
    else
#ifndef WIN32
        fd = open(filename, O_RDONLY, O_CREAT);
#else
        hFile = CreateFile((LPCSTR)filename,
                           GENERIC_READ,
                           0,
                           NULL,
                           OPEN_ALWAYS,
                           FILE_ATTRIBUTE_NORMAL,
                           NULL);
#endif
#ifndef WIN32
    if (fd < 0)
#else
    if (hFile == INVALID_HANDLE_VALUE)
#endif
        avr_error("failed to open the input file");
    else {
        Reset();

        SystemClock::Instance().Add(this);
    }
}

SerialTxFile::~SerialTxFile() {
#ifndef WIN32
    if ((fd != fileno(stdin)) &&
        (fd >= 0))
        close(fd);
#else
    if ((hFile != GetStdHandle(STD_INPUT_HANDLE)) &&
        (hFile != INVALID_HANDLE_VALUE))
        CloseHandle(hFile);
#endif
}

bool SerialTxFile::Sending(void) const {
    return (txState != TX_DISABLED);
}

int SerialTxFile::Step(bool &trueHwStep,
                       SystemClockOffset *timeToNextStepIn_ns) {

    if (Sending()) {
        SerialTxBuffered::Step(trueHwStep, timeToNextStepIn_ns);
    }

#ifndef WIN32
    pollfd cinfd[1];
    cinfd[0].fd = fd;
    cinfd[0].events = POLLIN;
    if (poll(cinfd, 1, 0) > 0) {
        unsigned char c;
        if (read(fd, &c, 1) > 0) {
            Send((unsigned char)c);
        }
    }
#else
    if (WaitForSingleObject(hFile, 0) == WAIT_OBJECT_0) {
        unsigned char c, b;
        if (ReadFile(hFile, &c, 1, &b, NULL)) {
            Send((unsigned char)c);
        }
    }
#endif

    if ( ! Sending()) // may have changed with Step() or Send()
        // Polling for new data every millisecond should be enough. If there's
        // more than one byte waiting, they're accepted at every serial clock
        // tick, which means, ten times faster than they can be sent.
        *timeToNextStepIn_ns = (SystemClockOffset)1000000;

    return 0;
}

// ===========================================================================
// ===========================================================================
// ===========================================================================


SerialTx::SerialTx(UserInterface *ui, const char *name, const char *baseWindow)
{
    std::ostringstream os;
    os << "create SerialTx " << name  << " " << baseWindow << std::endl;
    ui->Write(os.str());
    ui->AddExternalType(name, this);
    Reset();
}

void SerialTx::SetNewValueFromUi(const std::string &s) {
    std::cout << "SerialTx::SetNewValueFromUi >" << s << "<" << std::endl;
    if ( receiveInHex ) {
        unsigned char value;
        bool          rc;
        rc = StringToUnsignedChar( s.c_str(), &value, NULL, 16 );
        if ( !rc ) {
            std::cerr << "SerialTx::SetNewValueFromUi:: bad conversion" << std::endl;
        } else {
            // std::cerr << "SerialTx::Send " << std::hex << (unsigned int) value << std::endl;
            Send(value);
        }
    } else {
        if (s == "__SPACE__")  { Send(' '); }
        else 
        {
            for(unsigned int i=0; i < s.length(); i++)
            {
                Send(s[i]);
            }
        }
    }
}
