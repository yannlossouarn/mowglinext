# Unicore Runtime Profiles

Ces profils pilotent à la fois :

- les `LOG` activés sur le récepteur via `sensors/unicore/configure_receiver.sh`
- les diagnostics ROS 2 activés par `sensors/unicore/start_gps.sh`

Le profil par défaut est `normal`.

## Profils

| Profil | Usage recommandé | Logs principaux | Logs avancés |
| --- | --- | --- | --- |
| `normal` | runtime Nav2 / production légère | `PVTSLNA`, `GPGGA`, `BESTNAVA`, `GNHPR`, `RTKSTATUSA`, `RTCMSTATUSA` | aucun |
| `debug` | essais terrain / Foxglove | profil `normal` | `BESTSATA`, `SATSINFOA`, `AGCA`, `HWSTATUSA`, `JAMSTATUSA`, `FREQJAMSTATUSA`, `GSV` |
| `survey` | analyse GNSS avancée | profil `debug`, mais plus lent | `OBSVMCMPB` en mode `hybrid/binary` si explicitement activé |
| `high_precision` | précision max / tuning RTK | profil `debug` | `CONFIG PVTALG MULTI`, `CONFIG RTCMDECAUTO ENABLE`, `CONFIG RTCMPHASERATE POSITIVE`, `CONFIG RTCMCLOCKOFFSET ENABLE`, `OBSVMCMPB` optionnel |

## Périodes par défaut

| Profil | `main` | `bestnav` | `diagnostic` | `satellite` | `rf` | `raw` |
| --- | --- | --- | --- | --- | --- | --- |
| `normal` | `0.2` | `0.2` | `1` | `1` | `1` | `5` |
| `debug` | `0.2` | `0.2` | `1` | `1` | `1` | `5` |
| `survey` | `1` | `1` | `1` | `2` | `2` | `5` |
| `high_precision` | `0.1` | `0.1` | `1` | `1` | `1` | `5` |

Notes :

- `normal` reste léger par défaut.
- `OBSVMCMPB` reste désactivé par défaut. Il est destiné au `survey/debug` avancé, pas au runtime Nav2.
- Les résumés de diagnostics `GPS: raw observations` n’existent que si `UNICORE_ENABLE_RAW_OBSERVATIONS=true` et `UNICORE_OUTPUT_FORMAT=hybrid` ou `binary`.
- `high_precision` ajoute des commandes dépendantes du firmware N4 ; elles sont volontairement limitées à ce profil.

## Variables `.env`

Exemple recommandé pour MowgliNext en runtime standard :

```env
UNICORE_PROFILE=normal
UNICORE_TARGET_BAUD=921600
UNICORE_MAIN_LOG_PERIOD=0.2
UNICORE_BESTNAV_LOG_PERIOD=0.2
UNICORE_DIAGNOSTIC_LOG_PERIOD=1
UNICORE_SATELLITE_LOG_PERIOD=1
UNICORE_RF_LOG_PERIOD=1
UNICORE_RAW_LOG_PERIOD=5
UNICORE_ENABLE_SATELLITES=false
UNICORE_ENABLE_RF=false
UNICORE_ENABLE_JAMMING=false
UNICORE_ENABLE_RAW_OBSERVATIONS=false
```

Exemple terrain / debug Foxglove :

```env
UNICORE_PROFILE=debug
UNICORE_ENABLE_SATELLITES=true
UNICORE_ENABLE_RF=true
UNICORE_ENABLE_JAMMING=true
```

Exemple précision max :

```env
UNICORE_PROFILE=high_precision
UNICORE_MAIN_LOG_PERIOD=0.1
UNICORE_BESTNAV_LOG_PERIOD=0.1
UNICORE_ENABLE_SATELLITES=true
UNICORE_ENABLE_RF=true
UNICORE_ENABLE_JAMMING=true
```

Exemple `survey` avec observations compressées binaires :

```env
UNICORE_PROFILE=survey
UNICORE_OUTPUT_FORMAT=hybrid
UNICORE_ENABLE_RAW_OBSERVATIONS=true
UNICORE_RAW_LOG_PERIOD=5
```

## Recommandations

- `normal` : recommandé pour le runtime Nav2 quotidien
- `debug` : recommandé pour les essais terrain et l’analyse Foxglove
- `survey` : recommandé pour l’analyse GNSS avancée et les captures ponctuelles
- `high_precision` : recommandé pour les campagnes RTK exigeantes après validation firmware
