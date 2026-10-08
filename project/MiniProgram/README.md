# 微信小程序本地控制

## 使用条件

1. ESP-01S 烧录 `ESP8266_Firmware/wifi_bridge/wifi_bridge.ino`。
2. STM32 烧录当前 Keil 工程。
3. 手机手动连接 Wi-Fi `Gimbal-8266`，密码 `<LOCAL_WIFI_PASSWORD>`。
4. 微信开发者工具导入 `MiniProgram/gimbal-control`。

项目使用测试 AppID，并在开发设置中关闭请求域名校验。该方式适合个人开发、真机调试和现场局域网控制；正式发布的小程序需要 HTTPS 合法域名或云端服务。

## HTTP 接口

| 操作 | 地址 |
| --- | --- |
| 左右扫描 | `GET http://192.168.4.1/api/scan?mode=horizontal` |
| 上下扫描 | `GET http://192.168.4.1/api/scan?mode=vertical` |
| 全范围扫描 | `GET http://192.168.4.1/api/scan?mode=all` |
| 停止并回中 | `GET http://192.168.4.1/api/scan?mode=stop` |
| 查询状态 | `GET http://192.168.4.1/api/status` |

手机浏览器也可以直接打开 `http://192.168.4.1` 使用内置控制页面。
