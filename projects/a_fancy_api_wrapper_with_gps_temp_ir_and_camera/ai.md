```cpp
        if (data.endsWith("</!END BLOCK!>") || accumulatedData.indexOf("</") != -1 || accumulatedData.endsWith("<")) {
          Serial.println("Valid base64 image start received");
          accumulatedData += data;  // Start new accumulation
          // Append to existing base64 data
          accumulatedData += data;
          base64ArrayIndex = (base64ArrayIndex + 1) % MAX_BASE64_ARRAY;
          // Store the complete data
          base64Array[base64ArrayIndex] = accumulatedData;
          Serial.println(accumulatedData);
          Serial.print("Stored in slot: ");
          Serial.println(base64ArrayIndex);
          Serial.print("Total data length: ");
Serial.println(accumulatedData.length());
          // Clear the accumulation buffer
          accumulatedData = "";
          base64DataSendDone = true;
        } else if (data.startsWith("<!START BLOCK!>") && accumulatedData.length() == 0) {
          accumulatedData = data;
          base64DataSendDone = false;
        } else if (!base64DataSendDone) {
          accumulatedData += data;
        } else {
          Serial.println("Received non-base64 data: ");
          Serial.println(data);
        }
      }
```
The data should be 
`<!START BLOCK!> data:image/jpeg;base64,/9j/4AA/....</!END BLOCK!>`
But

```t
15:15:37.845 -> ..{"location":"臺北市士林區","weather":"陰","temperature":"36.1","humidity":"54","dailyHigh":"36.3","dailyLow":"27.2"}
15:16:40.561 -> {"location":"臺北市士林區","weather":"陰","temperature":"36.1","humidity":"54","dailyHigh":"36.3","dailyLow":"27.2"}
15:17:40.516 -> {"location":"臺北市士林區","weather":"陰","temperature":"36.1","humidity":"54","dailyHigh":"36.3","dailyLow":"27.2"}
15:18:23.603 -> Reading HUB 8735 Data!
15:18:27.860 -> Received data length: 39039
15:18:42.224 -> {"location":"臺北市士林區","weather":"陰","temperature":"36.1","humidity":"54","dailyHigh":"36.3","dailyLow":"27.2"}
15:19:42.919 -> {"location":"臺北市士林區","weather":"陰","temperature":"36.1","humidity":"54","dailyHigh":"36.3","dailyLow":"27.2"}
15:20:45.645 -> {"location":"臺北市士林區","weather":"陰","temperature":"36.1","humidity":"54","dailyHigh":"36.3","dailyLow":"27.2"}
15:21:44.083 -> Reading HUB 8735 Data!
```

How can I fix it?