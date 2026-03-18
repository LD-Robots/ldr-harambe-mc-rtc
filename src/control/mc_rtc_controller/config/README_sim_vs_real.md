# BWC Config: Simulare vs Real

## Observer Pipeline

### Simulare (mc_mujoco)
```yaml
ObserverPipelines:
  name: MainObserverPipeline
  gui: true
  observers:
    - type: Encoder
    - type: BodySensor
      update: true
      bodySensor: FloatingBase
```
BodySensor FloatingBase = ground truth din MuJoCo (pozitia exacta a floating base).

### Real (robot fizic)
```yaml
ObserverPipelines:
  name: MainObserverPipeline
  gui: true
  observers:
    - type: Encoder
    - type: KinematicInertial
      update: true
      config:
        bodySensor: PelvisIMU
```
KinematicInertial = estimeaza floating base din encodere + orientarea IMU (PelvisIMU pe torso).

## Senzori disponibili

| Senzor | Simulare | Real |
|--------|----------|------|
| Encodere (joint positions) | Da (MuJoCo) | Da (EtherCAT) |
| FloatingBase (ground truth pose) | Da (mc_mujoco injecteaza) | **NU** |
| PelvisIMU (gyro + accelerometer) | Da (MuJoCo sensor) | Da (BNO055 pe torso jos) |
| TorsoIMU (gyro + accelerometer) | Da (MuJoCo sensor) | Da (BNO055 pe torso sus) |
| Force sensors (picioare) | Simulate (dar nu folosite) | **NU** |

## Ce NU se schimba intre sim si real

- `OverwriteConfigKeys: [NoSensors]` — ramane, nu avem force sensors
- `enableZmpFeedback: false` — ramane
- `refComZ`, `midToFootTranss`, impedance gains — raman la fel
- `harambe.json` — acelasi fisier (contine definitii pentru toti senzorii)

## Pasi pentru deploy pe real

1. Schimba observer pipeline (BodySensor → KinematicInertial)
2. Verifica ca IMU-ul publica date pe topic-ul corect
3. Porneste cu robotul in harness (suspendat)
4. Tuneaza PD gains separat (fara MuJoCo damping pe real)
