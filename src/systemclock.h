 /*
 ****************************************************************************
 *
 * simulavr - A simulator for the Atmel AVR family of microcontrollers.
 * Copyright (C) 2001, 2002, 2003   Klaus Rudolph		
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ****************************************************************************
 */
#ifndef SYSTEMCLOCK
#define SYSTEMCLOCK
#include <map>
#include <vector>

#include "avrdevice.h"

class SystemClock: public multimap<unsigned long, SimulationMember *> {
    private:
        SystemClock(); //never !
        SystemClock(const SystemClock &);

    protected:
        unsigned long long currentTime;
        vector<SimulationMember*> asyncMembers;

    public:
        //SystemClock();

        void Add(SimulationMember *dev);
        void AddAsyncMember(SimulationMember *dev);
        int Step(bool &untilCoreStepFinished);
        void IncrTime(long long of) { currentTime+= of; }
        unsigned long long GetCurrentTime();
        void Endless();
        static SystemClock& Instance();
        void Rescedule( SimulationMember *sm, unsigned long long newTime);
};

#endif
