----------------------------------------------------------------------------------
-- Company: ETIS
-- Engineer: Mathieu Hannoun
--
-- Create Date: 25.05.2018 09:39:24
-- Design Name:
-- Module Name: custom_serializer - Behavioral
-- Project Name: Custom SERDES
-- Target Devices: Zedboard
-- Tool Versions:
-- Description: Serialize 32 bit word into NB_DATA_PORT bit word
-- Send them one by one at each rising edge of the counter clock
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

entity custom_serializer is
    Generic ( NB_DATA_PORT : INTEGER := 8);
    Port ( finished : out STD_LOGIC := '1';
           counter_clk : in STD_LOGIC;
           fifo_data_in : in STD_LOGIC_VECTOR (31 downto 0);
           gpio_data_out : out STD_LOGIC_VECTOR (NB_DATA_PORT - 1 downto 0));
end custom_serializer;

architecture Behavioral of custom_serializer is
signal currentStep : INTEGER := -1;
signal stepForWriting : INTEGER := 0;
signal overFlowFlag : STD_LOGIC := '0';
begin


overFlowFlag <= '0' when ((32 mod NB_DATA_PORT) = 0) else
                '1';

-- Number of step necessary for a writing, take in account overflow
stepForWriting <= (32 / NB_DATA_PORT)  when overFlowFLag = '0' else
                  (32 / NB_DATA_PORT) + 1;



-- Responsible for the output selection, split the data received from the fifo and output it
-- If the receiver never requested information the output is unknown
outputSelection : process (currentStep)
    begin
    if (currentStep = stepForWriting - 1) and (overFlowFlag = '1') then
        -- The most significant bits of the output is the least significant bits of the input
        -- The rest of the word is filled with '0'
        gpio_data_out((NB_DATA_PORT - 1) downto ((NB_DATA_PORT) - (32 mod NB_DATA_PORT))) <= fifo_data_in((32 mod NB_DATA_PORT) - 1 downto 0);
        -- TODO CHECK FOR MISTAKE HERE MAYBE -1 IN PLACE OF -2
        gpio_data_out(((NB_DATA_PORT - 2) - (32 mod NB_DATA_PORT)) downto 0 ) <= (others => '0');
    elsif currentStep >= 0 then
        gpio_data_out <= fifo_data_in((31 - (currentStep * NB_DATA_PORT)) downto (32 - ((currentStep + 1) * NB_DATA_PORT)));
    else
        gpio_data_out <= (others => 'Z');
    end if;
end process;

-- Responsible for the incrementation of the current step and generate a request signal for the fifo
stepIncrement : process (counter_clk)
begin
    if rising_edge(counter_clk) then
        if currentStep = stepForWriting - 1 then
            -- Reset current step, -1 when finished/inactive
            currentStep <= -1;
            finished <= '1';
        else
            currentStep <= currentStep + 1;
            finished <= '0';
        end if;
    end if;
end process;

end Behavioral;
