# Cloudflare Tunnel Setup for CASE Robot

Using Cloudflare Tunnel instead of ngrok (better for long-running sessions!)

## Setup in Google Colab

### Cell 1: Install Dependencies

```python
# Install Personaplex and WebSocket libraries
!git clone https://github.com/kyutai-labs/moshi.git
!cd moshi && pip install -e .
!pip install websockets numpy

# Install Cloudflare Tunnel (cloudflared)
!wget https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64
!chmod +x cloudflared-linux-amd64
!mv cloudflared-linux-amd64 /usr/local/bin/cloudflared
```

### Cell 2: Start WebSocket Server

```python
# Run the Personaplex WebSocket server on port 8998
!python personaplex_server_cloudflare.py
```

This will start the server and show:
```
✅ Server running on ws://0.0.0.0:8998
Waiting for ESP32 connections...
```

### Cell 3: Start Cloudflare Tunnel (Run in Parallel)

**Important: Run this in a SEPARATE cell while server is running**

```python
# Expose port 8998 via Cloudflare Tunnel
!cloudflared tunnel --url http://127.0.0.1:8998
```

**Output will show something like:**
```
2024-03-07 19:45:23 INF +--------------------------------------------------------------------------------------------+
2024-03-07 19:45:23 INF |  Your quick Tunnel has been created! Visit it at (it may take some time to be reachable):  |
2024-03-07 19:45:23 INF |  https://abc-def-123.trycloudflare.com                                                     |
2024-03-07 19:45:23 INF +--------------------------------------------------------------------------------------------+
```

**Copy the URL:** `https://abc-def-123.trycloudflare.com`

## Update ESP32 Code

### Convert HTTPS to WebSocket (WSS)

**Cloudflare gives you:**
```
https://abc-def-123.trycloudflare.com
```

**Convert to WebSocket URL for ESP32:**
```
wss://abc-def-123.trycloudflare.com/audio
```

**Update `case_voice.ino`:**
```cpp
const char* COLAB_SERVER_URL = "wss://abc-def-123.trycloudflare.com/audio";
```

## Advantages Over ngrok

| Feature | Cloudflare Tunnel | ngrok (Free) |
|---------|-------------------|--------------|
| **Time limit** | None | 2 hours |
| **URL changes** | Each session | Each session |
| **Request limit** | None | 40/min |
| **Reliability** | High | Medium |
| **Speed** | Fast | Fast |
| **Setup** | Easy | Easy |

## Port Configuration

**Make sure ports match:**

1. **WebSocket server** runs on `8998` (or change in code)
2. **Cloudflare tunnel** exposes `http://127.0.0.1:8998`
3. Both must use the **same port number**

If you want to use a different port (like 8765):

**In `personaplex_server_cloudflare.py`:**
```python
port = 8765  # Change this
```

**In Cloudflare tunnel:**
```python
!cloudflared tunnel --url http://127.0.0.1:8765
```

## Testing the Connection

### 1. Check Server is Running

Colab should show:
```
✅ Server running on ws://0.0.0.0:8998
```

### 2. Check Tunnel is Active

Cloudflare output should show:
```
https://xyz.trycloudflare.com
```

### 3. Test from ESP32

Upload code and check Serial Monitor:
```
Connecting to Colab server...
✅ Connected to Colab Personaplex server!
```

## Troubleshooting

### "Connection refused"

**Check:**
- Server is running (Cell 2)
- Tunnel is running (Cell 3)
- URL is correct in ESP32 code
- Used `wss://` not `https://`

### "Tunnel not starting"

**Try:**
```python
# Kill any existing tunnels
!pkill -9 cloudflared

# Restart tunnel
!cloudflared tunnel --url http://127.0.0.1:8998
```

### "WebSocket connection failed"

**Verify URL format:**
- ✅ Correct: `wss://abc-def.trycloudflare.com/audio`
- ❌ Wrong: `https://abc-def.trycloudflare.com/audio`
- ❌ Wrong: `ws://abc-def.trycloudflare.com/audio` (needs wss not ws)

### Colab Disconnects

**Cloudflare tunnel advantages:**
- Survives Colab reconnection (if backend still running)
- No 90-minute timeout like free ngrok
- More reliable for long sessions

**To keep Colab alive:**
- Use Colab Pro (recommended for Personaplex)
- Run periodic keep-alive cell
- Don't close browser tab

## Complete Workflow

1. **Start Colab with GPU runtime**
2. **Run Cell 1:** Install dependencies (one time)
3. **Run Cell 2:** Start WebSocket server (keeps running)
4. **Run Cell 3:** Start Cloudflare tunnel (keeps running)
5. **Copy URL** from Cloudflare output
6. **Convert** `https://` to `wss://` and add `/audio`
7. **Update ESP32 code** with new URL
8. **Upload to ESP32**
9. **Test:** Say "Hey CASE" and speak!

## Example Full URL

**Cloudflare gives:**
```
https://rapid-jokes-abc-123.trycloudflare.com
```

**Use in ESP32:**
```cpp
const char* COLAB_SERVER_URL = "wss://rapid-jokes-abc-123.trycloudflare.com/audio";
```

---

**Cloudflare Tunnel is the better choice for CASE robot!** More reliable, no time limits, perfect for development. 🚀
