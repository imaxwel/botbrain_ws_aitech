# go2w_pkg

Pacote específico do robô Go2W para a plataforma BotBrain. Este pacote fornece todos os componentes, configurações e interfaces necessários, projetados especificamente para o robô quadrúpede com rodas Unitree Go2W.

## Descrição

O pacote `go2w_pkg` serve como implementação específica do robô Unitree Go2W. Ele faz a ponte entre os tópicos DDS do SDK da Unitree e o ecossistema ROS 2, tratando da leitura de dados de sensores, escrita de comandos do robô, mapeamento de entradas de joystick/controle e streaming de vídeo. Todos os nós seguem o padrão de ciclo de vida do ROS 2.

## Estrutura de Diretórios

```
go2w_pkg/
├── config/                      # Arquivos de configuração
│   ├── camera_config.yaml      # TF e números de série das câmeras RealSense
│   ├── nav2_params.yaml        # Parâmetros de navegação do Nav2
│   └── pointcloud_to_laserscan_params.yaml  # Conversão de nuvem de pontos para laser scan
├── go2w_pkg/                    # Pacote Python
│   └── __init__.py
├── go2w_setup.bash              # Script de configuração de ambiente específico do Go2W
├── include/go2w_pkg/            # Diretório de cabeçalhos C++ (reservado)
├── launch/
│   ├── pc2ls.launch.py         # Launcher de conversão de nuvem de pontos para laser scan
│   └── robot_interface.launch.py  # Launcher principal da interface do robô Go2W
├── maps/                        # Mapas de navegação armazenados
├── meshes/                      # Arquivos de malha 3D para visualização do robô
│   ├── base.dae                # Malha da base do robô
│   ├── calf.stl / calf_mirror.stl
│   ├── foot.dae
│   ├── go2w_interface.stl      # Malha da interface Go2W
│   ├── hip.dae
│   ├── left_wheel.dae / right_wheel.dae  # Malhas das rodas (específicas do Go2W)
│   └── thigh.dae / thigh_mirror.dae
├── scripts/                     # Nós Python executáveis
│   ├── go2w_controller_commands.py  # Manipulador de botões do joystick/controle
│   ├── go2w_read.py            # Leitor de dados de sensores (nó de ciclo de vida)
│   ├── go2w_video_stream.py    # Publicador de stream de vídeo (nó de ciclo de vida)
│   └── go2w_write.py           # Escritor de comandos do robô (nó de ciclo de vida)
├── urdf/
│   └── robot.urdf              # Descrição do robô Go2W
├── xacro/                       # Arquivos XACRO (reservado)
├── CMakeLists.txt
└── package.xml
```

## Nós

### `go2w_read.py` — `robot_read_node`

Nó de ciclo de vida que faz a ponte entre os dados DDS da Unitree e os tópicos padrão do ROS 2.

**Inscrições:**
| Tópico | Tipo | Descrição |
|---|---|---|
| `/lf/sportmodestate` | `unitree_go/SportModeState` | Estado do modo esportivo (IMU, posição, velocidade) |
| `/lf/lowstate` | `unitree_go/LowState` | Estado de baixo nível (motores, bateria, BMS) |
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | Nuvem de pontos bruta do LiDAR |

**Publicações:**
| Tópico | Tipo | Descrição |
|---|---|---|
| `odom` | `nav_msgs/Odometry` | Odometria do robô |
| `imu/data` | `sensor_msgs/Imu` | Orientação, velocidade angular e aceleração linear da IMU |
| `imu_temp` | `std_msgs/Float32` | Temperatura da IMU |
| `pointcloud` | `sensor_msgs/PointCloud2` | Nuvem de pontos do LiDAR republicada (frame: `radar`) |
| `battery` | `sensor_msgs/BatteryState` | Tensão, corrente e nível de carga da bateria |
| `joint_states` | `sensor_msgs/JointState` | Posições e velocidades de todas as 16 juntas |
| `motor_temp` | `bot_custom_interfaces/MotorTemperature` | Temperaturas por motor |

Também transmite a transformada TF `{prefix}odom → {prefix}base_link`.

> Todos os tópicos listados acima são publicados sob o namespace do robô (`/{robot_name}/`), conforme configurado em `robot_config.yaml`.

---

### `go2w_write.py` — `lifecycle_robot_write_node`

Nó de ciclo de vida que traduz comandos ROS 2 em requisições para a API da Unitree e publica o status do robô.

**Inscrições:**
| Tópico | Tipo | Descrição |
|---|---|---|
| `cmd_vel_out` | `geometry_msgs/Twist` | Comandos de velocidade (encaminhados quando não há parada de emergência) |
| `/lf/sportmodestate` | `unitree_go/SportModeState` | Modo de operação e tipo de marcha atuais |
| `/api/sport/response` | `unitree_api/Response` | Resposta da API de esportes |
| `/api/robot_state/response` | `unitree_api/Response` | Resposta da API de estado do robô |
| `/api/vui/response` | `unitree_api/Response` | Resposta da API VUI (luzes) |

**Publicações:**
| Tópico | Tipo | Descrição |
|---|---|---|
| `/api/sport/request` | `unitree_api/Request` | Comandos do modo esportivo |
| `/api/robot_state/request` | `unitree_api/Request` | Comandos de estado do robô |
| `/api/vui/request` | `unitree_api/Request` | Comandos VUI (LED) |
| `robot_status` | `bot_custom_interfaces/RobotStatus` | Estado da parada de emergência e estado das luzes (2 Hz) |

**Serviços:**
| Serviço | Tipo | Descrição |
|---|---|---|
| `mode` | `bot_custom_interfaces/Mode` | Alternar modo: `damp`, `balance_stand`, `stand_up`, `stand_down` |
| `speed_level` | `bot_custom_interfaces/SpeedLevel` | Definir nível de velocidade no intervalo `[-1.0, 1.0]` |
| `current_mode` | `bot_custom_interfaces/CurrentMode` | Obter string do modo atual |
| `emergency_stop` | `std_srvs/SetBool` | Ativar/desativar parada de emergência (para o movimento e amortecer o robô) |
| `light_control` | `bot_custom_interfaces/LightControl` | Definir brilho do LED `[1–10]` ou desligar |

---

### `go2w_controller_commands.py` — `controller_commands_node`

Nó de ciclo de vida que mapeia pressionamentos de botões do controle físico para chamadas de serviço do robô a 10 Hz.

**Mapeamento de Botões:**
| Combinação de Botões | Ação |
|---|---|
| `start` | `balance_stand` (ou sair do modo de pose se estiver no modo 2) |
| `L2 + A` | Alternar `stand_up` / `stand_down` |
| `L2 + B` | `damp` |
| `L2 + X` | `recovery_stand` |

---

### `go2w_video_stream.py` — `robot_video_stream`

Nó de ciclo de vida que captura o stream de câmera H.264 embarcada do Go2W via GStreamer e o publica como tópicos de imagem ROS 2 a 24 fps.

- **Fonte do stream:** Multicast UDP `230.1.1.1:1720` na interface `eno1`
- **Resolução:** 1280×720

**Publicações:**
| Tópico | Tipo | Descrição |
|---|---|---|
| `camera` | `sensor_msgs/Image` | Imagem BGR bruta (1280×720) |
| `compressed_camera` | `sensor_msgs/CompressedImage` | Imagem comprimida JPEG (640×360, qualidade 20) |

> **Nota:** O nó de stream de vídeo está comentado em `robot_interface.launch.py` e deve ser habilitado manualmente se necessário.

---

## Arquivos de Launch

### `robot_interface.launch.py`

Inicia a interface completa do robô Go2W:
- `go2w_read_node`
- `go2w_write_node`
- `controller_commands_node`
- Conversão de nuvem de pontos para laser scan (`pc2ls.launch.py`)

O namespace do robô e o prefixo TF são lidos de `robot_config.yaml`.

### `pc2ls.launch.py`

Converte o tópico `pointcloud` para um `LaserScan` usando o pacote `pointcloud_to_laserscan`, com parâmetros de `config/pointcloud_to_laserscan_params.yaml`.

---

## Configuração

### `go2w_setup.bash`

Script de configuração de ambiente que:
1. Lê `network_interface` de `robot_config.yaml`
2. Define `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` e `CYCLONEDDS_URI` apontando para `cyclonedds_config.xml`
3. Configura a interface Ethernet com IP estático (`192.168.123.170/16`) para comunicação com o Go2W

Execute este script antes de iniciar a interface do robô:
```bash
source botbrain_ws/src/go2w_pkg/go2w_setup.bash
```

### `config/camera_config.yaml`

Define os frames TF e números de série das câmeras de profundidade RealSense:
- **Frontal:** D435i (serial `420122071637`), montada em `botbrain_base`
- **Traseira:** slot reservado, atualmente não configurado

---

## Dependências

- `rclpy`, `rclcpp`
- `unitree_go` — Tipos de mensagens do Unitree Go2 (`LowState`, `SportModeState`)
- `unitree_api` — Mensagens de requisição/resposta da API Unitree
- `bot_custom_interfaces` — Tipos de serviço e mensagem personalizados do BotBrain
- `joystick_bot` — Mensagens de estado dos botões do controle
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`
- `tf2_ros`
- `cv2` + `cv_bridge` (nó de stream de vídeo)
- `pointcloud_to_laserscan`

---

## Contribuição

1. **Criar Branch de Feature**: Crie um novo branch a partir de `dev` com o nome `feature/[nome_da_feature]`
2. **Desenvolvimento**: Faça suas alterações seguindo os padrões específicos do Go2W
3. **Testes**: Teste com o hardware real do robô Go2W
4. **Documentação**: Atualize este README com quaisquer novas funcionalidades ou alterações
5. **Submeter Pull Request**: Crie um pull request para o branch `dev`

---

**Nota:** Este pacote foi desenvolvido especificamente para o robô Unitree Go2W. Certifique-se de que o robô esteja ligado, a interface Ethernet esteja corretamente configurada via `go2w_setup.bash` e a ponte DDS do SDK da Unitree esteja em execução antes de iniciar.
