# nao-demo

```bash
./start_sim.bash
```

<br>

### Face

```bash
curl localhost:5000/face_emotion -X PUT -d "RED"    # or "1"
curl localhost:5000/face_emotion -X PUT -d "GREEN"  # or "2"
curl localhost:5000/face_emotion -X PUT -d "BLUE"   # or "3"
```

### handlwave

```bash
curl localhost:5000/hand_wave_enable -X PUT -d "1"  # You can put any value.
```

### tai_chi

```bash
curl localhost:5000/tai_chi_enable -X PUT -d "1"  # You can put any value.
```

### wipe_forehead

```bash
curl localhost:5000/wipe_enable -X PUT -d "1"  # You can put any value.
```

<br>