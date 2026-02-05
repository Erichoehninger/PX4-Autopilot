


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
