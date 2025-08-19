    if (enableHub8735 == true) {
      if (itemEntered) {
        if (lastCaptureTime + captureInterval < currentMillis && autoCapture) {
          if (debug) Serial.println("Taking pics!");
          H87_Serial.println("<!CAPTURE /!>");
          lastCaptureTime = currentMillis;  // Update the last capture time
        } else if (!autoCapture && !imageCaptured) {
          if (debug) {
            Serial.println("Auto recapture is disabled.");
          }
          H87_Serial.println("<!CAPTURE /!>");
          imageCaptured = true;
        }
      }
      ///*
      if (H87_Serial.available()) {
        pullingHub8735Data = true;
        if (debug) {
          Serial.println("Reading HUB 8735 Data!");
        }

        while (H87_Serial.available()) {
          String chunk = H87_Serial.readStringUntil('\n');
          chunk.trim();

          if (chunk.length() == 0) {
            if (debug2) Serial.println("DISCARDED (empty chunk).");
            continue;  // Ignore empty lines
          }



          if (debug2) {
            Serial.print("RAW CHUNK (len ");
            Serial.print(chunk.length());
            Serial.print(", in_progress=");
            Serial.print(base64DataInProgress);
            Serial.print("): [");
            Serial.print(chunk.substring(0, min((int)chunk.length(), 80)) + (chunk.length() > 80 ? "..." : ""));  // Print only first 80 chars
            Serial.println("]");
          }


          if (chunk.indexOf("<!START BLOCK!>") != -1) {
            if (!base64DataInProgress) {
              if (debug2) Serial.println("?you are in the first if process!");
              base64DataInProgress = true;
              accumulatedData = chunk;
              if (debug) Serial.println("Started accumulating base64 data.");
            } else {
              if (debug) Serial.println("Warning: Unexpected START marker while already in progress. Resetting accumulation.");
              accumulatedData = chunk;  // Reset and treat this as the new start
            }
            // IMPORTANT: Don't 'continue' here immediately. Let the current chunk be processed
            // by the 'if (base64DataInProgress)' block below, as it might contain the END marker too.
          } else {
            if (debug2) Serial.println("yeah no worky");
          }

          // Now, if we are currently in a Base64 data block (or just entered it)
          if (base64DataInProgress) {
            if (debug2) Serial.println("?you are in the second if process!");

            // Append chunk if it wasn't the *initial* START marker chunk (already handled `accumulatedData = chunk;`)
            // This is implicitly handled now that we don't have an early 'continue'.
            // The `accumulatedData` already holds the current chunk if it contained a START marker.
            // For subsequent chunks in the block, `accumulatedData` will grow.

            int startIndex = accumulatedData.indexOf("<!START BLOCK!>");
            int endIndex = accumulatedData.indexOf("</!END BLOCK!>");

            if (debug2) Serial.println(startIndex != -1 && endIndex != -1 && endIndex > startIndex);
            if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
              // Found both markers and END is after START
              String completeBase64Image = accumulatedData.substring(startIndex + 16, endIndex);

              // Validate the extracted string (optional but recommended)
              if (completeBase64Image.startsWith("data:image/jpeg;base64,") || completeBase64Image.startsWith("/9j/")) {
                base64ArrayIndex = (base64ArrayIndex + 1) % MAX_BASE64_ARRAY;
                base64Array[base64ArrayIndex] = completeBase64Image;
                imageCaptured = true;  // Mark that an image is ready for sending

                if (debug) {
                  Serial.printf("📷 Stored Base64 image in slot %d, length %d\n", base64ArrayIndex, completeBase64Image.length());
                }
                if (debug2) {  // This `debug2` block WILL activate if a complete image is stored
                  Serial.println("------------------------------------------------");
                  Serial.print("Stored in base64Array[");
                  Serial.print(base64ArrayIndex);
                  Serial.println("]:");
                  Serial.println(base64Array[base64ArrayIndex].substring(0, min((int)base64Array[base64ArrayIndex].length(), 100)) + "...");  // Print only first 100 chars
                }
              } else {
                if (debug) Serial.println("Warning: Extracted string did not look like valid Base64 image. Discarding.");
              }

              // IMPORTANT: Handle any residual data *after* the END marker in the same chunk.
              // If the END marker is at the very end of the chunk, this will be empty.
              // If there's garbage after the END marker, this captures it for the next loop to discard.
              String residualData = accumulatedData.substring(endIndex + 14);  // +14 for </!END BLOCK!> length
              if (debug2 && residualData.length() > 0) {
                Serial.print("RESIDUAL DATA AFTER END MARKER: [");
                Serial.print(residualData);
                Serial.println("]");
              }

              // Reset for next image
              accumulatedData = residualData;  // Keep residual data for next round, it might be the start of next garbage/message
              base64DataInProgress = false;

            } else {
              // Still accumulating inside a block, no END marker yet or markers are malformed.
              if (debug2) {
                Serial.print("Still accumulating (IN_BASE64_BLOCK), total accumulated: ");
                Serial.println(accumulatedData.length());
              }
            }
          } else {
            // We are NOT in a Base64 block and this chunk is NOT a START marker.
            // This chunk is therefore pure garbage and should be discarded.
            if (debug2) {
              Serial.print("DISCARDED (not in block, not a marker): [");
              Serial.print(chunk.substring(0, min((int)chunk.length(), 80)) + (chunk.length() > 80 ? "..." : ""));  // Print only first 80 chars
              Serial.println("]");
            }
            accumulatedData = "";  // Keep accumulatedData clean if not in a block.
          }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      //*/
      vTaskDelay(pdMS_TO_TICKS(10));
      pullingHub8735Data = false;
    }