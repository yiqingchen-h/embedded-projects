#include "board_storage.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/mman.h>
#endif

/* 在本测试进程内映射 Flash 地址，运行实际存储代码，不访问真实板卡。 */
#define FLASH_BASE 0x08000000UL
#define FLASH_OPTR_DBANK (1UL << 22U)
#define FLASH_BANK_1 1U
#define FLASH_TYPEERASE_PAGES 2U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0U
#define FLASH_FLAG_ALL_ERRORS UINT32_MAX
/* HAL 替身仅注入失败阶段；地址、页号和实际编程顺序由正式源码计算。 */
typedef enum {HAL_OK, HAL_ERROR} HAL_StatusTypeDef;
typedef struct {uint32_t TypeErase, Banks, Page, NbPages;} FLASH_EraseInitTypeDef;
static struct {uint32_t OPTR;} flash_registers;
#define FLASH (&flash_registers)
static uint16_t flash_kb = 128;
#define FLASHSIZE_BASE (&flash_kb)
static unsigned checks, unlocks, erases, programs, locks;
static bool fail_unlock, fail_erase, fail_lock, corrupt;
static unsigned fail_program;
static uint32_t hal_error, written_base;
#define CHECK(x) do {++checks; assert(x);} while (0)
#define __HAL_FLASH_CLEAR_FLAG(flags) do {CHECK((flags)==FLASH_FLAG_ALL_ERRORS);} while (0)

/** @brief 模拟解锁失败。 @return 指定的 HAL 状态。 */
static HAL_StatusTypeDef
HAL_FLASH_Unlock(void) {++unlocks; return fail_unlock?HAL_ERROR:HAL_OK;}
/** @brief 提供注入的 Flash 错误码。 @return 原始错误位。 */
static uint32_t
HAL_FLASH_GetError(void) {return hal_error;}
/** @brief 核对实际页号后擦除测试内存。 @param erase 擦除参数。 @param error 失败页。 @return 注入的结果。 */
static HAL_StatusTypeDef
HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error)
{
    ++erases;
    uint32_t page_size = BOARD_FLASH_DUAL_BANK?2048U:4096U;
    written_base = FLASH_BASE + erase->Page * page_size;
    CHECK(erase->Banks==FLASH_BANK_1 && erase->TypeErase==FLASH_TYPEERASE_PAGES);
    CHECK(written_base==BOARD_STORAGE_BASE || written_base==BOARD_STORAGE_BASE+BOARD_STORAGE_SLOT_SIZE);
    CHECK(erase->NbPages==BOARD_STORAGE_SLOT_SIZE/page_size);
    *error=fail_erase?erase->Page:UINT32_MAX;
    if (fail_erase) {return HAL_ERROR;}
    memset((void *)(uintptr_t)written_base,255,BOARD_STORAGE_SLOT_SIZE);
    return HAL_OK;
}
/** @brief 模拟双字编程和指定双字失败。 @param type 类型。 @param address 地址。 @param word 双字。 @return 编程结果。 */
static HAL_StatusTypeDef
HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t word)
{
    CHECK(type==FLASH_TYPEPROGRAM_DOUBLEWORD && address==written_base+8U*programs);
    ++programs;
    if (programs==fail_program) {return HAL_ERROR;}
    memcpy((void *)(uintptr_t)address,&word,sizeof(word));
    return HAL_OK;
}
/** @brief 模拟锁定失败或回读损坏。 @return 锁定结果。 */
static HAL_StatusTypeDef
HAL_FLASH_Lock(void)
{
    ++locks;
    if (corrupt) {*(uint8_t *)(uintptr_t)written_base^=1U;}
    return fail_lock?HAL_ERROR:HAL_OK;
}

/* 由回归脚本载入实际 board_storage.c，仅移除厂商 HAL include。 */
#include "storage_under_test.inc"

/** @brief 清除单次操作的故障和调用计数。 */
static void
reset_operation(void)
{
    unlocks=erases=programs=locks=fail_program=0;
    fail_unlock=fail_erase=fail_lock=corrupt=false;
    hal_error=0;
}

/** @brief 验证两种布局、错误阶段和槽边界。 @return 全部断言通过返回零。 */
int
main(void)
{
#ifdef _WIN32
    void *memory=VirtualAlloc((void *)(uintptr_t)FLASH_BASE,128U*1024U,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
#else
    void *memory=mmap((void *)(uintptr_t)FLASH_BASE,128U*1024U,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);
#endif
    CHECK(memory==(void *)(uintptr_t)FLASH_BASE);
    uint8_t data[32], readback[32];
    for (unsigned i=0;i<sizeof(data);++i) {data[i]=(uint8_t)(i+1);}
    flash_registers.OPTR=BOARD_FLASH_DUAL_BANK?0U:FLASH_OPTR_DBANK;
    CHECK(!board_storage_layout_valid());
    CHECK(strcmp(board_storage_status().result,"LAYOUT")==0);
    CHECK(!board_storage_read(0,readback,sizeof(readback)));
    CHECK(!board_storage_write(0,data,sizeof(data)));
    CHECK(unlocks==0 && erases==0 && programs==0);
    flash_registers.OPTR=BOARD_FLASH_DUAL_BANK?FLASH_OPTR_DBANK:0U;
    CHECK(board_storage_layout_valid());
    for (unsigned slot=0;slot<2;++slot)
    {
        reset_operation();
        CHECK(board_storage_write(slot,data,sizeof(data)));
        CHECK(board_storage_read(slot,readback,sizeof(readback)) && memcmp(data,readback,sizeof(data))==0);
        board_storage_status_t s=board_storage_status();
        CHECK(strcmp(s.result,"OK")==0 && s.flash_kb==128);
        CHECK(s.actual_dual_bank==s.build_dual_bank && s.address==BOARD_STORAGE_BASE+slot*BOARD_STORAGE_SLOT_SIZE);
        CHECK(unlocks==1 && erases==1 && programs==4 && locks==1);
    }
    reset_operation(); fail_unlock=true; hal_error=0x10;
    CHECK(!board_storage_write(1,data,sizeof(data)));
    CHECK(strcmp(board_storage_status().result,"UNLOCK")==0 && board_storage_status().hal_error==0x10);
    CHECK(erases==0 && locks==0);
    reset_operation(); fail_erase=true; hal_error=0x20;
    CHECK(!board_storage_write(1,data,sizeof(data)));
    CHECK(strcmp(board_storage_status().result,"ERASE")==0 && board_storage_status().hal_error==0x20);
    CHECK(board_storage_status().page_error==(BOARD_STORAGE_BASE+BOARD_STORAGE_SLOT_SIZE-FLASH_BASE)/(BOARD_FLASH_DUAL_BANK?2048U:4096U));
    CHECK(programs==0 && locks==1);
    reset_operation(); fail_program=2; hal_error=0x40;
    CHECK(!board_storage_write(1,data,sizeof(data)));
    CHECK(strcmp(board_storage_status().result,"PROGRAM")==0 && board_storage_status().hal_error==0x40);
    CHECK(board_storage_status().address==BOARD_STORAGE_BASE+BOARD_STORAGE_SLOT_SIZE+8U && locks==1);
    reset_operation(); corrupt=true;
    CHECK(!board_storage_write(1,data,sizeof(data)));
    CHECK(strcmp(board_storage_status().result,"VERIFY")==0);
    reset_operation(); fail_lock=true; hal_error=0x80;
    CHECK(!board_storage_write(1,data,sizeof(data)));
    CHECK(strcmp(board_storage_status().result,"LOCK")==0 && board_storage_status().hal_error==0x80);
    reset_operation();
    CHECK(!board_storage_write(2,data,sizeof(data)));
    CHECK(!board_storage_write(0,NULL,sizeof(data)));
    CHECK(!board_storage_write(0,data,0));
    CHECK(!board_storage_write(0,data,7));
    CHECK(strcmp(board_storage_status().result,"ARGUMENT")==0 && unlocks==0);
    printf("storage DBANK=%u: %u checks passed\n",BOARD_FLASH_DUAL_BANK,checks);
#ifdef _WIN32
    CHECK(VirtualFree(memory,0,MEM_RELEASE));
#else
    CHECK(munmap(memory,128U*1024U)==0);
#endif
    return 0;
}
