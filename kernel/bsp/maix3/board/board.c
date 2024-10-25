/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     lizhirui     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>

#include "board.h"
#include "tick.h"

#include "drv_uart.h"
#include "encoding.h"
#include "stack.h"
#include "sbi.h"
#include "riscv.h"
#include "stack.h"
#include "sysctl_boot.h"
#include "rtconfig.h"


#define MEMORY_RESERVED     0x1000






#define RTT_SYS_BASE        (CONFIG_MEM_RTSMART_BASE + CONFIG_RTSMART_OPENSIB_MEMORY_SIZE)
#define RTT_SYS_SIZE        ((((CONFIG_MEM_RTSMART_SIZE - CONFIG_RTSMART_OPENSIB_MEMORY_SIZE) / 1024) - 1) * 1024)

#define RAM_END             (RTT_SYS_BASE + RTT_SYS_SIZE - 4096)
#define RT_HEAP_SIZE        (CONFIG_MEM_RTSMART_HEAP_SIZE)

#define RT_HW_HEAP_BEGIN    ((void *)&__bss_end)
#define RT_HW_HEAP_END ((void *)(((rt_size_t)RT_HW_HEAP_BEGIN) + RT_HEAP_SIZE ))

#define RT_HW_PAGE_START    ((void *)((rt_size_t)RT_HW_HEAP_END + sizeof(rt_size_t)))
#define RT_HW_PAGE_END      ((void *)(RAM_END))




#ifdef RT_USING_USERSPACE
    #include "riscv_mmu.h"
    #include "mmu.h"
    #include "page.h"
    #include "lwp_arch.h"
    #include "ioremap.h"

    //这个结构体描述了buddy system的页分配范围
    rt_region_t init_page_region;
    //内核页表
    volatile rt_size_t MMUTable[__SIZE(VPN2_BIT)] __attribute__((aligned(4 * 1024)));
    rt_mmu_info mmu_info;
#endif

#ifdef CONFIG_AUTO_DETECT_DDR_SIZE


struct k230_ddr_size_tag_st{
  uint32_t flage;
  uint32_t shash;
  uint32_t resver;
  uint32_t ddr_size;
};
static uint32_t shash_len(const char *s,int len) {
  uint32_t v = 5381;
  int i = 0;

  for(i= 0; i < len; i++){
    v = (v << 5) + v + s[i];
  }
  return v;
}

rt_size_t get_ddr_phy_size(void)
{
    static unsigned int g_ddr_size = 0;

    struct k230_ddr_size_tag_st ddr_tag;
    int shash = 0;
    if(g_ddr_size == 0){
        memcpy(&ddr_tag, RTT_SYS_BASE - sizeof(ddr_tag), sizeof(ddr_tag));
        shash = ddr_tag.shash;
        ddr_tag.shash = 0;

        if(ddr_tag.flage == 0x5a5a5a5a && shash == shash_len((unsigned char *)&ddr_tag, sizeof(ddr_tag))){
            g_ddr_size = ddr_tag.ddr_size;
            //rt_kprintf("ddr size ok %x\n", g_ddr_size);
        }else {
            g_ddr_size = 0x20000000;
            //rt_kprintf("ddr size errror %x %x %x %x \n", g_ddr_size, ddr_tag.flage, shash,ddr_tag.ddr_size);
        }
    }

    return g_ddr_size;
}

rt_size_t  get_heap_size_by_ddr_size(void)
{
    rt_size_t ret = 0x2000000;
    switch (get_ddr_phy_size())
    {
        #ifdef CONFIG_MEM_RTSMART_HEAP_SIZE_512
        case 0x20000000:
            ret =  CONFIG_MEM_RTSMART_HEAP_SIZE_512;
            break;
        #endif
        #ifdef CONFIG_MEM_RTSMART_HEAP_SIZE_1024
        case 0x40000000:
            ret =  CONFIG_MEM_RTSMART_HEAP_SIZE_1024;
            break;
        #endif
        #ifdef CONFIG_MEM_RTSMART_HEAP_SIZE_2048
        case 0x80000000:
            ret =  CONFIG_MEM_RTSMART_HEAP_SIZE_2048;
            break;
        #endif
        default:
            break;
    }
    return ret;
}
rt_size_t get_rtsmart_sys_size(void)
{
    rt_size_t ret = 0x10000000;
    switch (get_ddr_phy_size())
    {
        #ifdef CONFIG_MEM_RTSMART_SIZE_512
        case 0x20000000:
            ret =  CONFIG_MEM_RTSMART_SIZE_512;
            break;
        #endif
        #ifdef CONFIG_MEM_RTSMART_SIZE_1024
        case 0x40000000:
            ret =  CONFIG_MEM_RTSMART_SIZE_1024;
            break;
        #endif
        #ifdef CONFIG_MEM_RTSMART_SIZE_2048
        case 0x80000000:
            ret =  CONFIG_MEM_RTSMART_SIZE_2048;
            break;
        #endif
        default:
            break;
    }
    return ret;
}
rt_size_t get_mem_mmz_base(void)
{
    rt_size_t ret = 0x10000000;
    switch (get_ddr_phy_size())
    {
        #ifdef CONFIG_MEM_MMZ_BASE_512
        case 0x20000000:
            ret = CONFIG_MEM_MMZ_BASE_512;
            break;
        #endif
        #ifdef CONFIG_MEM_MMZ_BASE_1024
        case 0x40000000:
            ret = CONFIG_MEM_MMZ_BASE_1024;
            break;
        #endif
        #ifdef CONFIG_MEM_MMZ_BASE_2048
        case 0x80000000:
            ret = CONFIG_MEM_MMZ_BASE_2048;
            break;
        #endif
        default:
            break;
    }
    return ret;
}
rt_size_t get_mem_mmz_size(void)
{
    rt_size_t ret = 0x10000000;
    switch (get_ddr_phy_size())
    {
        #ifdef CONFIG_MEM_MMZ_SIZE_512
        case 0x20000000:
            ret =  CONFIG_MEM_MMZ_SIZE_512;
            break;
        #endif

        #ifdef CONFIG_MEM_MMZ_SIZE_1024
        case 0x40000000:
            ret =  CONFIG_MEM_MMZ_SIZE_1024;
            break;
        #endif
        #ifdef CONFIG_MEM_MMZ_SIZE_2048
        case 0x80000000:
            ret =  CONFIG_MEM_MMZ_SIZE_2048;
            break;
        #endif
        default:
            break;
    }
    return ret;
}
#endif


//初始化BSS节区
void init_bss(void)
{
    unsigned int *dst;

    dst = &__bss_start;
    while (dst < &__bss_end)
    {
        *dst++ = 0;
    }
}

// #define MEM_RESVERD_SIZE    0x1000      /*隔离区*/
// #define MEM_IPCM_BASE 0x100000
// #define MEM_IPCM_SIZE 0xff000

// void init_ipcm_mem(void)
// {
//     rt_uint32_t *dst;
//     int i = 0;
//     dst = rt_ioremap((void *)(MEM_IPCM_BASE + MEM_IPCM_SIZE - MEM_RESVERD_SIZE - MEM_RESVERD_SIZE), MEM_RESVERD_SIZE);
//     if(dst == RT_NULL) {
//         rt_kprintf("ipcm ioremap error\n");
//     }
//     rt_memset((void *)dst, 0, MEM_RESVERD_SIZE);
//     for(i = 0; i < (0x1000 / 4); i++) {
//         if(dst[i] != 0) {
//             rt_kprintf("memest error addr:%p value:%d\n", &dst[i], dst[i]);
//         }
//     }
//     rt_iounmap((void *)dst);
// }

static void __rt_assert_handler(const char *ex_string, const char *func, rt_size_t line)
{
    rt_kprintf("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);
    // asm volatile("ebreak":::"memory");
    while (1){
        asm volatile("wfi");
    }
}

//BSP的C入口
void primary_cpu_entry(void)
{
    extern void entry(void);

    //初始化BSS
    init_bss();
    //关中断
    rt_hw_interrupt_disable();
    rt_assert_set_hook(__rt_assert_handler);
    //启动RT-Thread Smart内核
    entry();
}

#define IOREMAP_SIZE (1ul << 30)


//这个初始化程序由内核主动调用，此时调度器还未启动，因此在此不能使用依赖线程上下文的函数
void rt_hw_board_init(void)
{

#ifdef RT_USING_USERSPACE
    // rt_hw_mmu_map_init actually allocate a region for ioremap,
    // set (0xC000-0000 to 0xFFFF-FFFF) as IOREMAP(or kernel virt addr allocation) region
    // any peripherals in this region should be accessed after ioremap
    rt_hw_mmu_map_init(&mmu_info, (void *)(0x100000000UL - IOREMAP_SIZE), IOREMAP_SIZE, (rt_size_t *)MMUTable, 0);
    init_page_region.start = (rt_size_t)RT_HW_PAGE_START;
    init_page_region.end = (rt_size_t)RT_HW_PAGE_END;
    //rt_kprintf("s=%lx %lx \n",init_page_region.start, init_page_region.end);
    rt_page_init(init_page_region);
    rt_hw_mmu_kernel_map_init(&mmu_info, 0x00000000UL, (0x100000000UL - IOREMAP_SIZE));

    //将第3GB MMIO区域设置为无Cache与Strong Order访存模式
    MMUTable[2] &= ~PTE_CACHE;
    MMUTable[2] &= ~PTE_SHARE;
    MMUTable[2] |= PTE_SO;

    rt_hw_mmu_switch((void *)MMUTable);
#endif

#ifdef RT_USING_HEAP
    // rt_kprintf("heap: [0x%08x - 0x%08x]\n", (rt_ubase_t) RT_HW_HEAP_BEGIN, (rt_ubase_t) RT_HW_HEAP_END);
    /* initialize memory system */
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

// #if  MEM_IPCM_SIZE >  MEM_RESVERD_SIZE
//     init_ipcm_mem();
//     /* initalize interrupt */
// #endif


    rt_hw_interrupt_init();

    /* initialize hardware interrupt */

    rt_hw_tick_init();

#ifdef RT_USING_CONSOLE
    rt_hw_uart_init();
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif /* RT_USING_CONSOLE */

#ifdef RT_USING_HEAP
    rt_kprintf("heap: [0x%08x - 0x%08x], size %d KB\n", (rt_ubase_t) RT_HW_HEAP_BEGIN, (rt_ubase_t) RT_HW_HEAP_END, (rt_ubase_t)RT_HEAP_SIZE / 1024);
#endif

#ifdef RT_USING_USERSPACE
    rt_kprintf("page: [0x%08x - 0x%08x], size %d KB\n", (rt_ubase_t) RT_HW_PAGE_START, (rt_ubase_t) RT_HW_PAGE_END, (rt_ubase_t)((rt_ubase_t) RT_HW_PAGE_END - (rt_ubase_t) RT_HW_PAGE_START) / 1024);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

void rt_hw_cpu_reset(void)
{
    // sbi_shutdown();
    sysctl_boot_reset_soc();
    while(1);
}
MSH_CMD_EXPORT_ALIAS(rt_hw_cpu_reset, reboot, reset machine);
