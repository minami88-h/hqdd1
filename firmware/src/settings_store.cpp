#include "settings_store.h"
#include "build_id.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

// STM32G431KB: 2KBページ。ページ62 = 0x0801F000(ID保存はページ63)。
static const uint32_t STORE_ADDR = 0x0801F000UL;
static const uint32_t STORE_PAGE = 62;
static const uint32_t MAGIC = 0x5E77C0DFUL;

// G4のフラッシュはダブルワード(8B)単位でしか書けないので、サイズは8の倍数にする。
struct Blob {
  uint32_t magic;
  uint32_t build;  // 書き込んだ時のBUILD_ID。違えば(=書き込み直し)無効
  float v[SETTINGS_NPARAM];
  uint32_t check;
};
static_assert(sizeof(Blob) % 8 == 0, "Blob size must be a multiple of 8");

static uint32_t checksum(const Blob& b) {
  const uint32_t* w = reinterpret_cast<const uint32_t*>(&b);
  uint32_t x = 0xC3C3C3C3UL;
  for (size_t i = 0; i < (sizeof(Blob) - sizeof(uint32_t)) / 4; i++) x ^= w[i] + (uint32_t)i;
  return x;
}

bool settings_load(float v[SETTINGS_NPARAM]) {
  const Blob* b = reinterpret_cast<const Blob*>(STORE_ADDR);
  if (b->magic != MAGIC || b->build != BUILD_ID || b->check != checksum(*b)) return false;
  for (int i = 0; i < SETTINGS_NPARAM; i++) if (!isfinite(b->v[i])) return false;
  memcpy(v, b->v, sizeof(b->v));
  return true;
}

static bool erasePage() {
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = STORE_PAGE;
  erase.NbPages = 1;
  uint32_t pageError = 0;
  return HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;
}

bool settings_erase() {
  HAL_FLASH_Unlock();
  bool ok = erasePage();
  HAL_FLASH_Lock();
  float tmp[SETTINGS_NPARAM];
  return ok && !settings_load(tmp);
}

bool settings_save(const float v[SETTINGS_NPARAM]) {
  Blob b;
  b.magic = MAGIC;
  b.build = BUILD_ID;
  memcpy(b.v, v, sizeof(b.v));
  b.check = checksum(b);

  HAL_FLASH_Unlock();
  bool ok = erasePage();
  for (size_t i = 0; ok && i < sizeof(Blob) / 8; i++) {
    uint64_t dw;
    memcpy(&dw, reinterpret_cast<uint8_t*>(&b) + i * 8, 8);
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, STORE_ADDR + i * 8, dw) == HAL_OK;
  }
  HAL_FLASH_Lock();
  // 書いた内容が読み戻せるか確認
  float r[SETTINGS_NPARAM];
  return ok && settings_load(r) && memcmp(r, v, sizeof(r)) == 0;
}
