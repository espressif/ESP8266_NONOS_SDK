# Support Policy for ESP8266 NonOS

Starting from December 2019, 

* We will not add any new features to the ESP8266 NonOS SDK.
* We will only fix critical bugs in the ESP8266 NonOS SDK.
* We will only maintain the master branch of ESP8266 NonOS SDK, which is a continuously bug-fix version based on v3.0. This means:
	* All other released branches will not be updated.
	* All the future versions will be released from only the master branch mentioned above.
* It is suggested that the [ESP8266_RTOS_SDK](https://github.com/espressif/ESP8266_RTOS_SDK), instead of ESP8266 NonOS SDK, be used for your projects.

The latest ESP8266_RTOS_SDK allows users to develop applications using an architecture that are compatible with the SDKs of all Espressif chips, including ESP8266 series, ESP32 series, and the upcoming new series of chips. Switching to ESP8266_RTOS_SDK will helps users to:

* Eliminate the necessity to maintain more than one applications (for different chips), thus greatly reducing maintenance costs.
* Easily switch to other Espressif chips in the future for enhanced flexibility, less dependency, and reduced time-to-market.

Thank you for your interest in Espressif products.

# ESP8266 NonOS 支持政策

自 2019 年 12 月起，我们将：

* 停止为 ESP8266 NonOS 新增任何功能。
* 仅修复 ESP8266 NonOS 的关键 bug。
* 所有更新仅在 master 分支进行，即基于 v3.0.0 的持续 bug 修复版本。这意味着：
	* 其他任何 release 分支均不再提供维护；
	* 所有更新均将通过上述 master 分支发布。
* 建议客户使用新版 [ESP8266_RTOS_SDK](https://github.com/espressif/ESP8266_RTOS_SDK)。

简单来说，新版 ESP8266_RTOS_SDK 可帮助客户避免对单一 SDK 的依赖，允许客户应用程序同时兼容多款乐鑫芯片，包括 ESP8266 系列、ESP32 系列以及未来发布的新产品。使用 ESP8266_RTOS_SDK 允许客户：

* 避免同时维护针对不同芯片的多套应用程序，从而降低维护成本。
* 未来可轻松切换至其他乐鑫芯片，从而提高灵活性、降低对单一芯片的依赖，并缩短上市时间。

感谢大家对乐鑫的支持与关注。

# ESP8266_NONOS_SDK

All documentations @ http://espressif.com/en/support/download/documents?keys=&field_type_tid%5B%5D=14


## Notes ##
Please add `user_pre_init()` in your project, which will be called before `user_init()`. And you MUST call `system_partition_table_regist()` in `user_pre_init` to register your project partition table.  

The following partition address CAN NOT be modified, and you MUST give the correct address. They are retated to the flash map, please refer to [ESP8266 SDK Getting Started Guide](https://www.espressif.com/sites/default/files/documentation/2a-esp8266-sdk_getting_started_guide_en.pdf) or [ESP8266 SDK 入门指南](https://www.espressif.com/sites/default/files/documentation/2a-esp8266-sdk_getting_started_guide_cn.pdf).  
  
- SYSTEM\_PARTITION\_BOOTLOADER  
- SYSTEM\_PARTITION\_OTA_1  
- SYSTEM\_PARTITION\_OTA_2  
- SYSTEM\_PARTITION\_SYSTEM_PARAMETER  

If you donot use Non-FOTA bin, eagle.irom0.text.bin and irom0.text MUST be downloaded the fixed address, which also can be found in [ESP8266 SDK Getting Started Guide](https://www.espressif.com/sites/default/files/documentation/2a-esp8266-sdk_getting_started_guide_en.pdf) or [ESP8266 SDK 入门指南](https://www.espressif.com/sites/default/files/documentation/2a-esp8266-sdk_getting_started_guide_cn.pdf), and you can define their partition type after `SYSTEM_PARTITION_CUSTOMER_BEGIN`.