----------------------------------------------------------------------------------
-- Company:
-- Engineer: Mathieu Hannoun
--
-- Create Date: 28.05.2018 09:44:52
-- Design Name:
-- Module Name: custom_deserializer - Behavioral
-- Project Name: Custom SERDES
-- Target Devices: Zedboard
-- Tool Versions:
-- Description: Deserialize NB_DATA_PORT bits word into a 32 bits word
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
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity custom_deserializer is
    Generic ( NB_DATA_PORT : INTEGER := 8);
    Port ( finished : out STD_LOGIC;
           new_data_in_clk : in STD_LOGIC;
           data_in : in STD_LOGIC_VECTOR (NB_DATA_PORT - 1 downto 0);
           data_out : out STD_LOGIC_VECTOR (31 downto 0));
end custom_deserializer;


architecture Behavioral of custom_deserializer is

-- Output start at high impedance state
signal temp_out_vector : STD_LOGIC_VECTOR (31 downto 0) := "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ";
signal current_step : INTEGER := -1;
signal stepForWriting : INTEGER := 0;
signal overFlowFlag : STD_LOGIC := '0';
begin

overFlowFlag <= '0' when ((32 mod NB_DATA_PORT) = 0) else
                '1';

-- Number of step necessary for a writing, take in account overflow
stepForWriting <= (32 / NB_DATA_PORT)  when overFlowFLag = '0' else
                  (32 / NB_DATA_PORT) + 1;


concatenation_process : process (current_step)
begin
    -- In case of NB_DATA_PORT not divider of 32
    if (current_step = stepForWriting - 1) and (overFlowFlag = '1') then
        temp_out_vector((32 mod NB_DATA_PORT) - 1 downto 0) <=  data_in((NB_DATA_PORT - 1) downto ((NB_DATA_PORT) - (32 mod NB_DATA_PORT)));

    elsif current_step >= 0 then
        temp_out_vector((31 - (NB_DATA_PORT * current_step)) downto (32 - (NB_DATA_PORT * (current_step + 1)))) <= data_in;
    end if;

end process;


stepIncrement : process (new_data_in_clk)
begin
    if current_step = stepForWriting - 1 then -- Deserialization is finished
            data_out <= temp_out_vector;
            current_step <= -1;                      -- -1 is a waiting state
            finished <= '1';
    end if;
    if new_data_in_clk = '1' then
        if current_step = - 1 then
            current_step <= 0;
            finished <= '0';
--        elsif current_step = stepForWriting - 1 then -- Deserialization is finished
--            data_out <= temp_out_vector;
--            current_step <= -1;                      -- -1 is a waiting state
--            finished <= '1';
        else
            current_step <= current_step + 1;
        end if;
    else
        if current_step = -1 then
            data_out <= temp_out_vector;            -- Used only for initialization
            finished <= '1';
        end if;
    end if;
end process;

end Behavioral;
