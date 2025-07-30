# 存取 GPS 訊號與氣象局的資料

## 這個程式依賴的服務
- [Main API](https://github.com/hpware/weather-api-server.git)
- [氣象局 API Key](https://opendata.cwa.gov.tw)


## Stack
- Typescript
- Bun
- CWA API 服務
- ESP32
- DHT
- TinyGPS++
- ArduinoJson
- ArduinoFetch

## API 
### /weather/[Latitude]/[Longitude]

主要連結的程式

測試: ```curl https://[你的伺服器]/weather/[latitude]/[longitude]``` 

如果顯示 `error: "ERR_REMOTE_API_RATE_LIMIT"`，代表沒有加 API KEY 或是你的 API 的額度使用完了

### / 
404

