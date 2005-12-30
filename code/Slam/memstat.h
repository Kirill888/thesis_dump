#ifndef __MEM_STAT_H__
#define __MEM_STAT_H__

#ifdef USE_MEMSTAT

extern long memstat_count_odo;
extern long memstat_count_obs;
extern long memstat_count_mapNode;
extern long memstat_count_mapEntry;
extern long memstat_count_mapData;
extern long memstat_count_subMap;

#define MEM_STAT_ADD(varName)    ++memstat_count_##varName;
#define MEM_STAT_REMOVE(varName) --memstat_count_##varName;

extern void print_memstat(FILE *f);

#else

#define MEM_STAT_ADD(varName)
#define MEM_STAT_REMOVE(varName)

#define print_memstat(f)

#endif


#endif
