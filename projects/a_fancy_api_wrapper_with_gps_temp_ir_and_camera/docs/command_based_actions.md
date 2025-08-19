# 指令式系統
HUB8735 使用指令式系統 (來自 HTML 的格式)

## <!CAPTURE! />

### Demo:
```cpp
H87_Serial.println("<!CAPTURE! />");
```

## <!FLASHLIGHT!>{數字}</!FLASHLIGHT!>

### Demo:
```cpp
H87_Serial.print("<!FLASHLIGHT!>");
H87_Serial.print(ledPowerOnPoint);
H87_Serial.println("</!FLASHLIGHT!>");
```