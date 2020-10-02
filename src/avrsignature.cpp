/*
 ****************************************************************************
 *
 * simulavr - A simulator for the Atmel AVR family of microcontrollers.
 * Copyright (C) 2001, 2002 - 2012   Klaus Rudolph & other
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
#include "avrsignature.h"


// Idea taken from http://www.gamedev.net/topic/197291-creating-an-enum-to-string-look-up-table/
template<typename T_Key, typename T_Value>
class InitMap : public std::map<T_Key, T_Value> {
    public:
        InitMap& operator<< ( const std::pair<T_Key, T_Value>& v ) {
            this->insert( v );
            return *this;
        }
};

//! Map signature to device names (in lower case).
std::map<unsigned int, std::string> AvrSignatureToNameMap = InitMap<unsigned int, std::string>()
// MARK start Do not edit between this marker and the ending marker! This lines will be auto generated by script!
// MODE s2n
// TEMPLATE        << std::make_pair<unsigned int, std::string>(%(signature)s, "%(name)s")
        << std::make_pair<unsigned int, std::string>(0x1e930b, "attiny85")
        << std::make_pair<unsigned int, std::string>(0x1e930c, "attiny84")
        << std::make_pair<unsigned int, std::string>(0x1e9311, "attiny88")
        << std::make_pair<unsigned int, std::string>(0x1e9303, "at90s8535")
        << std::make_pair<unsigned int, std::string>(0x1e950c, "atmega3290p")
        << std::make_pair<unsigned int, std::string>(0x1e9682, "at90usb646")
        << std::make_pair<unsigned int, std::string>(0x1e9682, "at90usb647")
        << std::make_pair<unsigned int, std::string>(0x1e9203, "at90s4433")
        << std::make_pair<unsigned int, std::string>(0x1e9801, "atmega2560")
        << std::make_pair<unsigned int, std::string>(0x1e9202, "at90s4434")
        << std::make_pair<unsigned int, std::string>(0x1e930a, "atmega88")
        << std::make_pair<unsigned int, std::string>(0x1e9006, "attiny15")
        << std::make_pair<unsigned int, std::string>(0x1e9005, "attiny12")
        << std::make_pair<unsigned int, std::string>(0x1e9007, "attiny13")
        << std::make_pair<unsigned int, std::string>(0x1e9003, "attiny10")
        << std::make_pair<unsigned int, std::string>(0x1e9004, "attiny11")
        << std::make_pair<unsigned int, std::string>(0x1e9504, "atmega3290")
        << std::make_pair<unsigned int, std::string>(0x1e9581, "at90can32")
        << std::make_pair<unsigned int, std::string>(0x1e9741, "atxmega128a1revd")
        << std::make_pair<unsigned int, std::string>(0x1e9482, "at90usb162")
        << std::make_pair<unsigned int, std::string>(0x1e9103, "at90s2343")
        << std::make_pair<unsigned int, std::string>(0x1e958a, "atmega32u2")
        << std::make_pair<unsigned int, std::string>(0x1e9602, "atmega64")
        << std::make_pair<unsigned int, std::string>(0x1e9587, "atmega32u4")
        << std::make_pair<unsigned int, std::string>(0x1e9307, "atmega8")
        << std::make_pair<unsigned int, std::string>(0x1e9541, "atxmega32a4")
        << std::make_pair<unsigned int, std::string>(0x1e9306, "atmega8515")
        << std::make_pair<unsigned int, std::string>(0x1e9405, "atmega169")
        << std::make_pair<unsigned int, std::string>(0x1e9301, "at90s8515")
        << std::make_pair<unsigned int, std::string>(0x1e9681, "at90can64")
        << std::make_pair<unsigned int, std::string>(0x1e9401, "atmega161")
        << std::make_pair<unsigned int, std::string>(0x1e9382, "at90usb82")
        << std::make_pair<unsigned int, std::string>(0x1e9402, "atmega163")
        << std::make_pair<unsigned int, std::string>(0x1e9404, "atmega162")
        << std::make_pair<unsigned int, std::string>(0x1e9001, "at90s1200")
        << std::make_pair<unsigned int, std::string>(0x1e9647, "atxmega64d4")
        << std::make_pair<unsigned int, std::string>(0x1e930f, "atmega88p")
        << std::make_pair<unsigned int, std::string>(0x1e960a, "atmega644p")
        << std::make_pair<unsigned int, std::string>(0x1e9205, "atmega48")
        << std::make_pair<unsigned int, std::string>(0x1e9701, "atmega103")
        << std::make_pair<unsigned int, std::string>(0x1e940a, "atmega164p")
        << std::make_pair<unsigned int, std::string>(0x1e9483, "at90pwm316")
        << std::make_pair<unsigned int, std::string>(0x1e9441, "atxmega16a4")
        << std::make_pair<unsigned int, std::string>(0x1e9802, "atmega2561")
        << std::make_pair<unsigned int, std::string>(0x1e9308, "atmega8535")
        << std::make_pair<unsigned int, std::string>(0x1e9008, "attiny9")
        << std::make_pair<unsigned int, std::string>(0x1e9746, "atxmega128a4")
        << std::make_pair<unsigned int, std::string>(0x1ea701, "atmega128rfa1")
        << std::make_pair<unsigned int, std::string>(0x1e974c, "atxmega128a1")
        << std::make_pair<unsigned int, std::string>(0x1e9843, "atxmega256a3b")
        << std::make_pair<unsigned int, std::string>(0x1e9742, "atxmega128a3")
        << std::make_pair<unsigned int, std::string>(0x1e9406, "atmega168")
        << std::make_pair<unsigned int, std::string>(0x1e910c, "attiny261")
        << std::make_pair<unsigned int, std::string>(0x1e9502, "atmega32")
        << std::make_pair<unsigned int, std::string>(0x1e9105, "at90s2333")
        << std::make_pair<unsigned int, std::string>(0x1e920d, "attiny4313")
        << std::make_pair<unsigned int, std::string>(0x1e9782, "at90usb1286")
        << std::make_pair<unsigned int, std::string>(0x1e9782, "at90usb1287")
        << std::make_pair<unsigned int, std::string>(0x1e9381, "at90pwm2")
        << std::make_pair<unsigned int, std::string>(0x1e9208, "attiny461")
        << std::make_pair<unsigned int, std::string>(0x1e9542, "atxmega32d4")
        << std::make_pair<unsigned int, std::string>(0x1e950b, "atmega329p")
        << std::make_pair<unsigned int, std::string>(0x1e9646, "atxmega64a4")
        << std::make_pair<unsigned int, std::string>(0x1e9508, "atmega324p")
        << std::make_pair<unsigned int, std::string>(0x1e9747, "atxmega128d4")
        << std::make_pair<unsigned int, std::string>(0x1e9206, "attiny45")
        << std::make_pair<unsigned int, std::string>(0x1e9207, "attiny44")
        << std::make_pair<unsigned int, std::string>(0x1e964e, "atxmega64a1")
        << std::make_pair<unsigned int, std::string>(0x1e9642, "atxmega64a3")
        << std::make_pair<unsigned int, std::string>(0x1e9744, "atxmega192a3")
        << std::make_pair<unsigned int, std::string>(0x1e974e, "atxmega192a1")
        << std::make_pair<unsigned int, std::string>(0x1e9383, "at90pwm2b")
        << std::make_pair<unsigned int, std::string>(0x1e9705, "atmega1284p")
        << std::make_pair<unsigned int, std::string>(0x1e9604, "atmega6490")
        << std::make_pair<unsigned int, std::string>(0x1e9503, "atmega329")
        << std::make_pair<unsigned int, std::string>(0x1e9514, "atmega328")
        << std::make_pair<unsigned int, std::string>(0x1e9505, "atmega325")
        << std::make_pair<unsigned int, std::string>(0x1e9702, "atmega128")
        << std::make_pair<unsigned int, std::string>(0x1e9442, "atxmega16d4")
        << std::make_pair<unsigned int, std::string>(0x1e9403, "atmega16")
        << std::make_pair<unsigned int, std::string>(0x1e9101, "at90s2313")
        << std::make_pair<unsigned int, std::string>(0x1e9201, "at90s4414")
        << std::make_pair<unsigned int, std::string>(0x1e9703, "atmega1280")
        << std::make_pair<unsigned int, std::string>(0x1e9704, "atmega1281")
        << std::make_pair<unsigned int, std::string>(0x1e940b, "atmega168p")
        << std::make_pair<unsigned int, std::string>(0x1e9389, "atmega8u2")
        << std::make_pair<unsigned int, std::string>(0x1e9511, "atmega324pa")
        << std::make_pair<unsigned int, std::string>(0x1e9781, "at90can128")
        << std::make_pair<unsigned int, std::string>(0x1e930d, "attiny861")
        << std::make_pair<unsigned int, std::string>(0x1e9489, "atmega16u2")
        << std::make_pair<unsigned int, std::string>(0x1e8f0a, "attiny4")
        << std::make_pair<unsigned int, std::string>(0x1e8f09, "attiny5")
        << std::make_pair<unsigned int, std::string>(0x1e9846, "atxmega256a1")
        << std::make_pair<unsigned int, std::string>(0x1e9603, "atmega649")
        << std::make_pair<unsigned int, std::string>(0x1e9842, "atxmega256a3")
        << std::make_pair<unsigned int, std::string>(0x1e9608, "atmega640")
        << std::make_pair<unsigned int, std::string>(0x1e9609, "atmega644")
        << std::make_pair<unsigned int, std::string>(0x1e910a, "attiny2313")
        << std::make_pair<unsigned int, std::string>(0x1e9109, "attiny26")
        << std::make_pair<unsigned int, std::string>(0x1e9108, "attiny25")
        << std::make_pair<unsigned int, std::string>(0x1e910b, "attiny24")
        << std::make_pair<unsigned int, std::string>(0x1e920a, "atmega48p")
// MARK end
;

//! Map device names (in lower case) to signature.
std::map<std::string, unsigned int> AvrNameToSignatureMap = InitMap<std::string, unsigned int>()
// MARK start Do not edit between this marker and the ending marker! This lines will be auto generated by script!
// MODE n2s
// TEMPLATE        << std::make_pair<std::string, unsigned int>("%(name)s", %(signature)s)
        << std::make_pair<std::string, unsigned int>("attiny85", 0x1e930b)
        << std::make_pair<std::string, unsigned int>("attiny84", 0x1e930c)
        << std::make_pair<std::string, unsigned int>("attiny88", 0x1e9311)
        << std::make_pair<std::string, unsigned int>("at90s8535", 0x1e9303)
        << std::make_pair<std::string, unsigned int>("atmega3290p", 0x1e950c)
        << std::make_pair<std::string, unsigned int>("at90usb646", 0x1e9682)
        << std::make_pair<std::string, unsigned int>("at90usb647", 0x1e9682)
        << std::make_pair<std::string, unsigned int>("at90s4433", 0x1e9203)
        << std::make_pair<std::string, unsigned int>("atmega2560", 0x1e9801)
        << std::make_pair<std::string, unsigned int>("at90s4434", 0x1e9202)
        << std::make_pair<std::string, unsigned int>("atmega88", 0x1e930a)
        << std::make_pair<std::string, unsigned int>("attiny15", 0x1e9006)
        << std::make_pair<std::string, unsigned int>("attiny12", 0x1e9005)
        << std::make_pair<std::string, unsigned int>("attiny13", 0x1e9007)
        << std::make_pair<std::string, unsigned int>("attiny10", 0x1e9003)
        << std::make_pair<std::string, unsigned int>("attiny11", 0x1e9004)
        << std::make_pair<std::string, unsigned int>("atmega3290", 0x1e9504)
        << std::make_pair<std::string, unsigned int>("at90can32", 0x1e9581)
        << std::make_pair<std::string, unsigned int>("atxmega128a1revd", 0x1e9741)
        << std::make_pair<std::string, unsigned int>("at90usb162", 0x1e9482)
        << std::make_pair<std::string, unsigned int>("at90s2343", 0x1e9103)
        << std::make_pair<std::string, unsigned int>("atmega32u2", 0x1e958a)
        << std::make_pair<std::string, unsigned int>("atmega64", 0x1e9602)
        << std::make_pair<std::string, unsigned int>("atmega32u4", 0x1e9587)
        << std::make_pair<std::string, unsigned int>("atmega8", 0x1e9307)
        << std::make_pair<std::string, unsigned int>("atxmega32a4", 0x1e9541)
        << std::make_pair<std::string, unsigned int>("atmega8515", 0x1e9306)
        << std::make_pair<std::string, unsigned int>("atmega169", 0x1e9405)
        << std::make_pair<std::string, unsigned int>("at90s8515", 0x1e9301)
        << std::make_pair<std::string, unsigned int>("at90can64", 0x1e9681)
        << std::make_pair<std::string, unsigned int>("atmega161", 0x1e9401)
        << std::make_pair<std::string, unsigned int>("at90usb82", 0x1e9382)
        << std::make_pair<std::string, unsigned int>("atmega163", 0x1e9402)
        << std::make_pair<std::string, unsigned int>("atmega162", 0x1e9404)
        << std::make_pair<std::string, unsigned int>("at90s1200", 0x1e9001)
        << std::make_pair<std::string, unsigned int>("atxmega64d4", 0x1e9647)
        << std::make_pair<std::string, unsigned int>("atmega88p", 0x1e930f)
        << std::make_pair<std::string, unsigned int>("atmega644p", 0x1e960a)
        << std::make_pair<std::string, unsigned int>("atmega48", 0x1e9205)
        << std::make_pair<std::string, unsigned int>("atmega103", 0x1e9701)
        << std::make_pair<std::string, unsigned int>("atmega164p", 0x1e940a)
        << std::make_pair<std::string, unsigned int>("at90pwm316", 0x1e9483)
        << std::make_pair<std::string, unsigned int>("atxmega16a4", 0x1e9441)
        << std::make_pair<std::string, unsigned int>("atmega2561", 0x1e9802)
        << std::make_pair<std::string, unsigned int>("atmega8535", 0x1e9308)
        << std::make_pair<std::string, unsigned int>("attiny9", 0x1e9008)
        << std::make_pair<std::string, unsigned int>("atxmega128a4", 0x1e9746)
        << std::make_pair<std::string, unsigned int>("atmega128rfa1", 0x1ea701)
        << std::make_pair<std::string, unsigned int>("atxmega128a1", 0x1e974c)
        << std::make_pair<std::string, unsigned int>("atxmega256a3b", 0x1e9843)
        << std::make_pair<std::string, unsigned int>("atxmega128a3", 0x1e9742)
        << std::make_pair<std::string, unsigned int>("atmega168", 0x1e9406)
        << std::make_pair<std::string, unsigned int>("attiny261", 0x1e910c)
        << std::make_pair<std::string, unsigned int>("atmega32", 0x1e9502)
        << std::make_pair<std::string, unsigned int>("at90s2333", 0x1e9105)
        << std::make_pair<std::string, unsigned int>("attiny4313", 0x1e920d)
        << std::make_pair<std::string, unsigned int>("at90usb1286", 0x1e9782)
        << std::make_pair<std::string, unsigned int>("at90usb1287", 0x1e9782)
        << std::make_pair<std::string, unsigned int>("at90pwm2", 0x1e9381)
        << std::make_pair<std::string, unsigned int>("attiny461", 0x1e9208)
        << std::make_pair<std::string, unsigned int>("atxmega32d4", 0x1e9542)
        << std::make_pair<std::string, unsigned int>("atmega329p", 0x1e950b)
        << std::make_pair<std::string, unsigned int>("atxmega64a4", 0x1e9646)
        << std::make_pair<std::string, unsigned int>("atmega324p", 0x1e9508)
        << std::make_pair<std::string, unsigned int>("atxmega128d4", 0x1e9747)
        << std::make_pair<std::string, unsigned int>("attiny45", 0x1e9206)
        << std::make_pair<std::string, unsigned int>("attiny44", 0x1e9207)
        << std::make_pair<std::string, unsigned int>("atxmega64a1", 0x1e964e)
        << std::make_pair<std::string, unsigned int>("atxmega64a3", 0x1e9642)
        << std::make_pair<std::string, unsigned int>("atxmega192a3", 0x1e9744)
        << std::make_pair<std::string, unsigned int>("atxmega192a1", 0x1e974e)
        << std::make_pair<std::string, unsigned int>("at90pwm2b", 0x1e9383)
        << std::make_pair<std::string, unsigned int>("atmega1284p", 0x1e9705)
        << std::make_pair<std::string, unsigned int>("atmega6490", 0x1e9604)
        << std::make_pair<std::string, unsigned int>("atmega329", 0x1e9503)
        << std::make_pair<std::string, unsigned int>("atmega328", 0x1e9514)
        << std::make_pair<std::string, unsigned int>("atmega325", 0x1e9505)
        << std::make_pair<std::string, unsigned int>("atmega128", 0x1e9702)
        << std::make_pair<std::string, unsigned int>("atxmega16d4", 0x1e9442)
        << std::make_pair<std::string, unsigned int>("atmega16", 0x1e9403)
        << std::make_pair<std::string, unsigned int>("at90s2313", 0x1e9101)
        << std::make_pair<std::string, unsigned int>("at90s4414", 0x1e9201)
        << std::make_pair<std::string, unsigned int>("atmega1280", 0x1e9703)
        << std::make_pair<std::string, unsigned int>("atmega1281", 0x1e9704)
        << std::make_pair<std::string, unsigned int>("atmega168p", 0x1e940b)
        << std::make_pair<std::string, unsigned int>("atmega8u2", 0x1e9389)
        << std::make_pair<std::string, unsigned int>("atmega324pa", 0x1e9511)
        << std::make_pair<std::string, unsigned int>("at90can128", 0x1e9781)
        << std::make_pair<std::string, unsigned int>("attiny861", 0x1e930d)
        << std::make_pair<std::string, unsigned int>("atmega16u2", 0x1e9489)
        << std::make_pair<std::string, unsigned int>("attiny4", 0x1e8f0a)
        << std::make_pair<std::string, unsigned int>("attiny5", 0x1e8f09)
        << std::make_pair<std::string, unsigned int>("atxmega256a1", 0x1e9846)
        << std::make_pair<std::string, unsigned int>("atmega649", 0x1e9603)
        << std::make_pair<std::string, unsigned int>("atxmega256a3", 0x1e9842)
        << std::make_pair<std::string, unsigned int>("atmega640", 0x1e9608)
        << std::make_pair<std::string, unsigned int>("atmega644", 0x1e9609)
        << std::make_pair<std::string, unsigned int>("attiny2313", 0x1e910a)
        << std::make_pair<std::string, unsigned int>("attiny26", 0x1e9109)
        << std::make_pair<std::string, unsigned int>("attiny25", 0x1e9108)
        << std::make_pair<std::string, unsigned int>("attiny24", 0x1e910b)
        << std::make_pair<std::string, unsigned int>("atmega48p", 0x1e920a)
// MARK end
;

// EOF
