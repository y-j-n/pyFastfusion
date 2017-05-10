/*
 * memory.cpp
 *
 *  Created on: Feb 27, 2013
 *      Author: steinbrf
 */

#include "memory.hpp"
#include <unistd.h>
#include <fstream>
#include <string>

#ifdef __APPLE__
#include <sys/sysctl.h>
#endif

#ifdef __APPLE__
long long getMemOSX(int type) {
//http://nadeausoftware.com/articles/2012/09/c_c_tip_how_get_physical_memory_size_system#sysctl
    int mib[2];
	mib[0] = CTL_HW;
    //mib[1] = HW_PHYSMEM;
    //unsigned int size = 0;  /* 32-bit */
	//size_t len = sizeof( size );

//http://stackoverflow.com/questions/583736/determine-physical-mem-size-programmatically-on-osx
    mib[1] = type;
    uint64_t size;
    size_t len = sizeof(size);

	if ( sysctl( mib, 2, &size, &len, NULL, 0 ) == 0 ) {
		return (size_t)size;
    } else {
        return 0L;  /* Failed? */
    }
}
#endif


long long getTotalSystemMemory()
{
#ifdef __APPLE__
#warning kludge for osx
    return getMemOSX(HW_MEMSIZE);
#else
    long long pages = sysconf(_SC_PHYS_PAGES);
    long long page_size = sysconf(_SC_PAGE_SIZE);
    return pages * page_size;
#endif
}

long long getAvailableSystemMemory()
{
#ifdef __APPLE__
#warning kludge for osx
    return getMemOSX(HW_USERMEM);
#else
    long long pages = sysconf(_SC_AVPHYS_PAGES);
    long long page_size = sysconf(_SC_PAGE_SIZE);
    return pages * page_size;
#endif
}

ProcessMemoryStats getProcessMemory(){
	std::fstream file("/proc/self/statm");
	ProcessMemoryStats result;
	file >> result.size;
	file >> result.resident;
	file >> result.shared;
	file >> result.trs;
	file >> result.drs;
	file >> result.lrs;
	file >> result.dt;
	result.pageSize = sysconf(_SC_PAGE_SIZE);
	return result;
}



