import numpy as np, requests, sounddevice as sd                              
sr = 16000                                                                   
print('Speak now! Recording 3 seconds...')                                   
audio = sd.rec(sr * 3, samplerate=sr, channels=1, dtype='int16')
sd.wait()                                                                    
print('Sending to Personaplex...')                        
r = requests.post('https://tions-vessel-mine-coding.trycloudflare.com/api/audio', data=audio.flatten().tobytes(), headers={'Content-Type': 'application/octet-stream'}, timeout=60)                             
print(f'Response: {r.status_code}, {len(r.content)} bytes')                  
if r.status_code == 200 and len(r.content) > 0:                              
    resp = np.frombuffer(r.content, dtype=np.int16)
    print(f'Playing CASE response ({len(resp)/sr:.1f}s)...')                 
    sd.play(resp, sr); sd.wait()                                             
else:                                                                        
    print(f'No audio response. Body: {r.text[:200]}')                       