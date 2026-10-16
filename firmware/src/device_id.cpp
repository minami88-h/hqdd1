#include "device_id.h"
#include "build_id.h"
#include <Arduino.h>
#include <string.h>

// STM32G431KB: 128KB flash, 2KBページ、シングルバンク。
// 最終ページ(63番, 0x0801F800)をID保存専用に使う。
// アプリは現状80KB強で、今後増えても十分な余裕がある場所。
static const uint32_t STORE_ADDR = 0x0801F800UL;
static const uint32_t STORE_PAGE = 63;
static const uint32_t MAGIC = 0xA5A5A5A5UL;

// 16バイト(ダブルワード2つ)構造体。G4のフラッシュはダブルワード単位でしか書けない。
// buildが現在のBUILD_IDと違えば(=プログラムを書き込み直したら)無効とみなし、
// コード上の既定ID(DEFAULT_DEVICE_ID)を使う。
struct StoreBlock {
  uint32_t magic;
  uint32_t build;
  uint8_t id;
  uint8_t check;  // id ^ 0xFF (簡易な破損検出。CRCは1バイトの保護には過剰)
  uint8_t reserved[6];
};

static uint8_t s_id = 0;

static bool readStore(uint8_t* outId) {
  const StoreBlock* stored = reinterpret_cast<const StoreBlock*>(STORE_ADDR);
  if (stored->magic != MAGIC || stored->build != BUILD_ID) return false;
  if (stored->check != (uint8_t)(stored->id ^ 0xFF)) return false;
  if (stored->id > 7) return false;
  *outId = stored->id;
  return true;
}

static void writeStore(uint8_t id) {
  StoreBlock block;
  block.magic = MAGIC;
  block.build = BUILD_ID;
  block.id = id;
  block.check = (uint8_t)(id ^ 0xFF);
  memset(block.reserved, 0xFF, sizeof(block.reserved));

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = STORE_PAGE;
  erase.NbPages = 1;
  uint32_t pageError = 0;
  HAL_FLASHEx_Erase(&erase, &pageError);

  static_assert(sizeof(StoreBlock) == 2 * sizeof(uint64_t), "StoreBlock must be exactly 16 bytes");
  for (int i = 0; i < 2; i++) {
    uint64_t doubleWord;
    memcpy(&doubleWord, reinterpret_cast<uint8_t*>(&block) + i * 8, 8);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, STORE_ADDR + i * 8, doubleWord);
  }

  HAL_FLASH_Lock();
}

void device_id_init() {
  uint8_t id;
  s_id = readStore(&id) ? id : DEFAULT_DEVICE_ID;
}

uint8_t device_id_get() {
  return s_id;
}

void device_id_set(uint8_t id) {
  if (id > 7 || id == s_id) return;
  s_id = id;
  writeStore(id);
}
