----------------------------------------------------------------------------------
-- Company:
-- Engineer: Mathieu Hannoun
--
-- Create Date: 04.06.2018 10:29:29
-- Design Name:
-- Module Name: curstom_ser_com_2 - Behavioral
-- Project Name: Custom SERDES
-- Target Devices: Zedboard
-- Tool Versions:
-- Description: This module control the serializer and the deserializer, this is a moore
-- finite state machine.
--
-- Dependencies:
--
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity curstom_ser_com_2 is
    Port ( new_data_available : in STD_LOGIC;
       main_clk     : in STD_LOGIC;
       ser_ena      : out STD_LOGIC := '0';
       des_ena      : out STD_LOGIC;
       error_flag   : out STD_LOGIC := '0';
       rst          : in STD_LOGIC;
       ser_finished : in STD_LOGIC;
       des_finished : in STD_LOGIC
       );
end curstom_ser_com_2;

architecture Behavioral of curstom_ser_com_2 is
-- TODO check if cleaning state is really necessary
type state is (idle, start, working, waiting_for_ser, waiting_for_des, cleaning);
signal current_state, futur_state : state;
begin


-- Transition function, refers to the schematic in the repport for more details
transition : process(current_state, new_data_available, ser_finished, des_finished)
begin
    case current_state is
        when idle =>
            if new_data_available = '1' then
                futur_state <= start;
            else
                futur_state <= idle;
            end if;
        when start =>
            futur_state <= working;
        when working =>
            futur_state <= waiting_for_des;
        when waiting_for_des =>
            if des_finished = '1' then
                futur_state <= waiting_for_ser;
            else
                futur_state <= waiting_for_des;
            end if;
        when waiting_for_ser =>
            if ser_finished = '1' then
                futur_state <= cleaning;
            else
                futur_state <= waiting_for_ser;
            end if;

        when cleaning =>
            futur_state <= idle;
    end case;
end process;

memorisation : process(main_clk, rst)
begin
    if rst = '1' then
        current_state <= idle;
    end if;
    if rising_edge(main_clk) then
        current_state <= futur_state;
    end if;
end process;

-- Generation function responsible for the output state
generation : process(current_state)
begin
    case current_state is
        when idle =>
            ser_ena     <= '0';
            des_ena     <= '0';
            error_flag  <= '0';

        when start =>
            ser_ena     <= '0';
            des_ena     <= '1';
            error_flag  <= '0';

        when working =>
            ser_ena     <= '1';
            des_ena     <= '0';
            error_flag  <= '0';

        when waiting_for_ser =>
            ser_ena     <= '0';
            des_ena     <= '0';
            error_flag  <= '0';

        when waiting_for_des =>
            ser_ena     <= '0';
            des_ena     <= '0';
            error_flag  <= '0';
        when cleaning =>
            ser_ena     <= '0';
            des_ena     <= '0';
            error_flag  <= '0';
    end case;
end process;

end Behavioral;
