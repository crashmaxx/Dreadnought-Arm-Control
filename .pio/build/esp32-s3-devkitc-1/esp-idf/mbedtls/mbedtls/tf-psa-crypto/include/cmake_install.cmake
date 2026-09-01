# Install script for directory: C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/VESC_Express_Encoder_CRSF_Control")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/Users/andre/.platformio/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/psa" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_adjust_auto_enabled.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_adjust_config_dependencies.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_adjust_config_derived.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_adjust_config_key_pair_types.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_adjust_config_synonyms.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_builtin_composites.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_builtin_key_derivation.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_builtin_primitives.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_compat.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_config.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_driver_common.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_driver_contexts_composites.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_driver_contexts_key_derivation.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_driver_contexts_primitives.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_driver_random.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_extra.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_platform.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_sizes.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_struct.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_types.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/psa/crypto_values.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/tf-psa-crypto" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/tf-psa-crypto/build_info.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/tf-psa-crypto/version.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/../drivers/builtin/include/mbedtls/config_adjust_legacy_crypto.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/../drivers/builtin/include/mbedtls/private_access.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/asn1.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/asn1write.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/base64.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/compat-3-crypto.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/constant_time.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/lms.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/md.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/memory_buffer_alloc.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/nist_kw.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/pem.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/pk.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/platform.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/platform_time.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/platform_util.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/psa_util.h"
    "C:/Users/andre/.platformio/packages/framework-espidf/components/mbedtls/mbedtls/tf-psa-crypto/include/mbedtls/threading.h"
    )
endif()

