# Smart Lock Moto - Firmware de Segurança Crítica

Este projeto implementa um sistema de trava inteligente via Bluetooth Low Energy (BLE) utilizando o **Zephyr RTOS**. O foco principal é a independência de hardware e a segurança ativa do condutor.

---

##  Arquitetura e Hardware
O firmware foi desenvolvido para ser multiplataforma, validado tanto no **nRF52832** quanto no **ESP32-C3**.

**Abstração de Hardware:** Todos os periféricos são definidos via **Device Tree Aliases**, garantindo que o código C não possua pinagens fixas (*hard-coded*).
* **Interceptor do Sensor de Tombamento:** * O hardware atua entre o sensor e a ECU.
    * **Operação Normal:** Repassa os 5V originais do sensor para a ECU.
    * **Bloqueio Ativo:** Chaveia o sinal para 1V (via divisor de tensão interno), simulando o estado de queda para a ECU e desligando o motor.

---

##  Lógica de Segurança (Fail-Safe)
A segurança é garantida por uma barreira lógica de software no controle de GPIO:

1. **Interlock de Ignição:** O pino `P0.04` monitora a ignição. Se ativa, o comando de bloqueio é impedido para evitar acionamentos em movimento.
2. **Bloqueio Pendente:** Se o usuário tentar trancar com a moto ligada, o sistema agenda o bloqueio (`RECEIVED_COMMAND`) para o instante em que a chave for desligada.
3. **Persistência (NVS):** O estado da trava é salvo em memória não volátil, protegendo contra reinicializações por perda de energia.

---

##  Interface Bluetooth (GATT)
**Service UUID:** `0x1523`

| Característica | UUID | Descrição |
| :--- | :--- | :--- |
| **STATUS** | `0x1524` | Notifica o estado (`LOCKED`, `UNLOCKED`, `RECEIVED_COMMAND`)  |
| **CTRL** | `0x1525` | Comandos: `LOCK`, `UNLOCK`, `TOGGLE` |
| **TIME_SYNC** | `0x1526` | Sincronização de timestamp UTC para auditoria |

---

##  Processo de Instalação e Gravação
Para o módulo personalizado nRF52832:
1. **Compilação:** Utilize a extensão nRF Connect no VS Code.
2. **Limpeza:** Realize o *Erase Chip* via **J-Flash Lite** para remover proteções de leitura.
3. **Gravação:** Grave o arquivo `.hex` gerado na pasta de build.

---
*Este projeto segue os princípios de escalabilidade e redução de custos de portabilidade*
