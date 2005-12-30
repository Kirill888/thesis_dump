#ifdef USE_MEMSTAT

#ifndef DEPEND
#include <stdio.h>
#endif

#include "memstat.h"

#include "map.h"
#include "DataStore.h"


long memstat_count_odo = 0;
long memstat_count_mapNode = 0;
long memstat_count_mapEntry = 0;
long memstat_count_mapData = 0;
long memstat_count_subMap = 0;

long memstat_size_odo        = sizeof(OdometryStoreNode);
long memstat_size_mapNode    = sizeof(struct Map::TreeNode);
long memstat_size_mapEntry   = sizeof(MapEntry);
long memstat_size_mapData    = sizeof(struct MapEntry::listEntry);

static void print_memstat(FILE* f, const char *name, int n, int sz){
  fprintf(f,"%s %03d(%db)", name, n, sz);
}

void print_memstat(FILE *f){
  fprintf(f,"MEM: ");

  print_memstat(f,"odo"  ,memstat_count_odo, memstat_size_odo);
  print_memstat(f,", map_n",memstat_count_mapNode, memstat_size_mapNode);
  print_memstat(f,", map_e",memstat_count_mapEntry, memstat_size_mapEntry);
  print_memstat(f,", map_d",memstat_count_mapData, memstat_size_mapData);
  print_memstat(f,", subm" ,memstat_count_subMap, 1);

  int total = memstat_count_odo      * memstat_size_odo
            + memstat_count_mapNode  * memstat_size_mapNode
            + memstat_count_mapEntry * memstat_size_mapEntry
            + memstat_count_mapData  * memstat_size_mapData;


  fprintf(f,"\nMEM: total = %09d bytes\n",total);
}


#endif
