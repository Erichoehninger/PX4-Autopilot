
# Global Drones TMR

Módulo de gerenciamento de tripla redundância para o projeto GD350.

### Instruções de Uso:

 - Compilação: `make GD_sitl none`
 - Criação de outras instâncias: `./build/GD_sitl_default/bin/px4 -i <numero_da_instancia>`
 - Definir PX4_COMM_ID no terminal PX4: `param set PX4_COMM_ID <numero_da_instancia>`
 - Uso do módulo dentro do terminal PX4: `sensor_can_publisher start <numero_de_envios>`
