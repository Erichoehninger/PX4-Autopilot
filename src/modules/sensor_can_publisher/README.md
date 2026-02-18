




# Global Drones TMR





Módulo de gerenciamento de tripla redundância para o projeto GD350.





### Instruções de Uso sem simulador:



- Compilação: `make GD_sitl none`

- Criação de outras instâncias: `./build/GD_sitl_default/bin/px4 -i <numero_da_instancia>`

- Definir SYS_PX4_COMM_ID no terminal PX4: `param set SYS_PX4_COMM_ID <numero_da_instancia>`

- Uso do módulo dentro do terminal PX4: `sensor_can_publisher start <numero_de_envios>`





### Instruções de Uso com simulador gazebo:

- Compilação: `make GD_sitl gz_x500`

- Criação de outras instâncias: ` PX4_SYS_AUTOSTART=4001 ./build/GD_sitl_default/bin/px4 -i <numero_da_instancia>`

- Definir SYS_PX4_COMM_ID no terminal PX4: `param set SYS_PX4_COMM_ID <numero_da_instancia>`

- Uso do módulo dentro do terminal PX4: `sensor_can_publisher start <numero_de_envios>`



### Instruções compilação para Pixhawk 6x Pro:

- Compilação: `make px4_fmu-v6x_default`



#### Observações:

- Sempre que o código PX4 for recompilado, é necessário reconfigurar o SYS_PX4_COMM_ID das instâncias configuradas

- Pixhawk 6x Pro roda com C++ reduzido. std::map, std::threads, std::mutex, etc não compilam.



#### Estado atual:



- O estado de odometria da controladora líder é publicado no tópico **`leader_publishable_info`** e consumido no módulo **`EKF2.cpp`**, por meio da função **`UpdateExtVisionSample()`**.

- Esses dados são recebidos pelas FCs servas e tratados como uma fonte de **External Vision**, sendo então fundidos ao EKF local de cada controladora com **alto peso** em relação às demais medições.

- Para que essa fusão ocorra corretamente, é necessário configurar os seguintes parâmetros:

-  **EKF2_EVP_NOISE = 0.01**

Atua como uma métrica de qualidade da medição de posição externa, influenciando diretamente o peso atribuído à odometria do líder durante a fusão.

-  **EKF2_EV_CTRL = 15**

Habilita a fusão completa dos dados de External Vision (posição, velocidade e yaw).

-  **EKF2_MULTI_IMU = 4**

Define o número máximo de instâncias de EKF baseadas em múltiplas IMUs.
