#include <jffs2/jffs2.h>

#include <common.h>
#include <command.h>
#include <nand.h>
#include "../common/tegra2_partitions.h"
#include "am_load_splash.h"

static void set_mtdparts_env(void) {
  if (!getenv("mtdparts")) {
    char env_str[256];
    int i;
    sprintf(env_str, "mtdparts=tegra_nand:");
    i = strlen(env_str);
    if(nvtegra_mtdparts_string(env_str + i, sizeof(env_str) - i)) {
      setenv("mtdparts", env_str);
    }
  }
}

void am_load_splash(void) {
#if defined(CONFIG_COLIBRI_T20) && defined(CONFIG_SPLASH_SCREEN)
  const char *partname = CONFIG_AM_SPLASH_PARTITION;
  struct mtd_device *dev;
  struct part_info *part;
  u8 pnum;
  size_t rsize;
  ulong addr;

#if 0
  if(getenv("splashimage") == NULL) {
    return;
  }
  addr = CONFIG_LOADADDR;
#else
  char *s;
  if((s = getenv("splashimage")) == NULL) {
    return;
  }
  if(strict_strtoul(s, 16, &addr) != 0) {
    return;
  }
#endif
  set_mtdparts_env();
  if(mtdparts_init())
    return;
  
  if(find_dev_and_part(partname, &dev, &pnum, &part))
    return;

  if (dev->id->type != MTD_DEV_TYPE_NAND) {
    return;
  }
  rsize = part->size;
  nand_read_skip_bad(&nand_info[dev->id->num], part->offset, &rsize, (u_char *)addr);
#endif
}
