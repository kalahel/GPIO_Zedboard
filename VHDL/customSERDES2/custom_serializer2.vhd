----------------------------------------------------------------------------------
-- Company:
-- Engineer: Mathieu Hannoun
--
-- Create Date: 04.06.2018 10:07:24
-- Design Name:
-- Module Name: custom_serializer_2 - Behavioral
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
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity custom_serializer_2 is
    Generic (   NB_DATA_PORT : INTEGER := 8
            );
    Port (      finished         : out STD_LOGIC := '1';
                ser_ena          : in STD_LOGIC;
                main_clk         : in STD_LOGIC;
                rst              : in STD_LOGIC;
                data_in     : in STD_LOGIC_VECTOR (31 downto 0);
                data_out    : out STD_LOGIC_VECTOR (NB_DATA_PORT - 1 downto 0)
                );
end custom_serializer_2;

architecture Behavioral of custom_serializer_2 is
signal current_step                          : INTEGER := -1;
signal stepForWriting                       : INTEGER := 0;
signal overFlowFlag, working, working_rst   : STD_LOGIC := '0';
begin


overFlowFlag <= '0' when ((32 mod NB_DATA_PORT) = 0) else
                '1';

-- Number of step necessary for a writing, take in account overflow
stepForWriting <= (32 / NB_DATA_PORT)  when overFlowFLag = '0' else
                  (32 / NB_DATA_PORT) + 1;



-- Responsible for the output selection, split the data received from the fifo and output it
-- If the receiver never requested information the output is unknown
outputSelection : process (current_step, main_clk, rst)
begin
    if rst = '1' then
        data_out <= (others => 'Z');
    end if;
    if falling_edge(main_clk) then
        if (current_step = stepForWriting - 1) and (overFlowFlag = '1') then
            -- The most significant bits of the output is the least significant bits of the input
            -- The rest of the word is filled with '0'
            data_out((NB_DATA_PORT - 1) downto ((NB_DATA_PORT) - (32 mod NB_DATA_PORT))) <= data_in((32 mod NB_DATA_PORT) - 1 downto 0);
            data_out(((NB_DATA_PORT - 2) - (32 mod NB_DATA_PORT)) downto 0 ) <= (others => '0');
        elsif (current_step >= 0) and (current_step < stepForWriting) then
            data_out <= data_in((31 - (current_step * NB_DATA_PORT)) downto (32 - ((current_step + 1) * NB_DATA_PORT)));
        else
            data_out <= (others => 'Z');
        end if;
    end if;
end process;

-- Responsible for the incrementation of the current step and generate a request signal for new data
stepIncrement : process (main_clk, working, rst)
begin
    if rst = '1' then
        current_step <= -1;
        working_rst <= '0';
        finished <= '1';
    end if;
    if falling_edge(main_clk) then
        if working = '1' then
            if current_step = stepForWriting then
                -- Reset current step, -1 when finished/inactive
                current_step <= -1;
                finished <= '1';
                working_rst <= '1';

            else
                current_step <= current_step + 1;
                finished <= '0';
                working_rst <= '0';
            end if;
        else
            working_rst <= '0';             -- This is mandatory to prevent locking in reset state
        end if;
    end if;
end process;

-- Working is a memorised signal used to mark when the deserializer is currently working
-- Activated only when receiving an impulse from the sercom
working_memorisation : process(rst, main_clk, working_rst, ser_ena)
begin
    if (rst = '1' or working_rst = '1') then
        working <= '0';
    end if;
    if(falling_edge(main_clk)) then
        if (ser_ena = '1') then
            working <= '1';
        end if;
    end if;
end process;

end Behavioral;
