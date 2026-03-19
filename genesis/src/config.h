/* Local config override for MegaWifi Performance Test Server */

#include_next "config.h"

/* Enable MegaWifi module */
#undef MODULE_MEGAWIFI
#define MODULE_MEGAWIFI 1

/* Set buffer size explicitly for performance testing */
#ifndef MW_BUFLEN
#define MW_BUFLEN 1460  /* Max Ethernet payload, allows testing large blocks */
#endif