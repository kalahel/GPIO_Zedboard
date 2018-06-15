----------------------------------------------------------------------------------
-- Company:
-- Engineer: Mathieu Hannoun
--
-- Create Date: 01.06.2018 15:18:33
-- Design Name:
-- Module Name: custom_deserializer_2 - Behavioral
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

entity custom_deserializer_2 is
    Generic ( NB_DATA_PORT      : INTEGER := 8
            );
Port ( finished : out STD_LOGIC := '1';
       main_clk             : in STD_LOGIC;
       rst                  : in STD_LOGIC;
       des_ena              : in STD_LOGIC;
       data_in              : in STD_LOGIC_VECTOR (NB_DATA_PORT - 1 downto 0);
       write_enable_probe   : out STD_LOGIC_VECTOR ((32 / NB_DATA_PORT) DOWNTO 0);
       data_out             : out STD_LOGIC_VECTOR (31 downto 0) := "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"
       );
end custom_deserializer_2;

architecture Behavioral of custom_deserializer_2 is

component custom_generic_register is
    Generic ( N : INTEGER := 8);
    Port (  data_in         : in    STD_LOGIC_VECTOR(N - 1 DOWNTO 0);
            data_out        : out   STD_LOGIC_VECTOR(N - 1 DOWNTO 0);
            write_enable    : in    STD_LOGIC;
            clk             : in    STD_LOGIC;
            rst             : in    STD_LOGIC
     );
end component custom_generic_register;


signal write_enable_s                           : STD_LOGIC_VECTOR ((32 / NB_DATA_PORT) DOWNTO 0);
signal current_step                             : INTEGER := -1;
signal stepForWriting                           : INTEGER;
signal overFlowFLag, working, working_rst       : STD_LOGIC := '0';
signal write_enable_base_s                      : BIT_VECTOR((32 / NB_DATA_PORT) DOWNTO 0);
signal overflowed_out_s                         : STD_LOGIC_VECTOR (NB_DATA_PORT -1 DOWNTO 0);
begin

overFlowFlag <= '0' when ((32 mod NB_DATA_PORT) = 0) else
                '1';

-- Number of step necessary for a writing, take in account overflow
stepForWriting <= (32 / NB_DATA_PORT)  when overFlowFLag = '0' else
                  (32 / NB_DATA_PORT) + 1;

-- Used later for shifting
write_enable_base_s <= (0 => '1',others => '0');

write_enable_probe <= write_enable_s;


-- Generate (32 / NB_DATA_PORT) - 1 register or (32 / NB_DATA_PORT) if NB_DATA_PORT is not a divider of 32
generation : for i in 0 to (32 / NB_DATA_PORT) generate
    divider : if (i <  (32 / NB_DATA_PORT)) generate
        reg : custom_generic_register GENERIC MAP (N => NB_DATA_PORT) PORT MAP(
            data_in             => data_in,
            data_out            => data_out(31 - (i * NB_DATA_PORT) DOWNTO (32 - ((i + 1) * NB_DATA_PORT))),
            write_enable        => write_enable_s(i),
            clk                 => main_clk,
            rst                 => rst
            );
    end generate divider;
    -- Register for overflowed bits
    overflow : if ((i =  (32 / NB_DATA_PORT)) and (not ((32 mod NB_DATA_PORT) = 0))) generate
        reg : custom_generic_register GENERIC MAP (N => NB_DATA_PORT) PORT MAP(
            data_in                                                                     => data_in,
--            data_out((NB_DATA_PORT -1) downto (NB_DATA_PORT - (32 mod NB_DATA_PORT)))   => data_out(((32 mod NB_DATA_PORT) - 1 ) downto 0),
            data_out                                                                    => overflowed_out_s,
            write_enable                                                                => write_enable_s(i),
            clk                                                                         => main_clk,
            rst                                                                         => rst
        );
        data_out(((32 mod NB_DATA_PORT) - 1 ) downto 0) <= overflowed_out_s((NB_DATA_PORT -1) downto (NB_DATA_PORT - (32 mod NB_DATA_PORT)));
    end generate overflow;
end generate generation;

-- Progression in the deserialisation step
step_progression : process(rst, main_clk, current_step, working)
begin
    if(rst = '1') then
        current_step <= -1;
        working_rst <= '0';
        finished <= '1';
        write_enable_s <= (others => '0');
    end if;
    if(rising_edge(main_clk)) then
        if (working = '1') then
            if(current_step = (stepForWriting)) then                        -- Deserialization has ended
                finished <= '1';
                current_step <= -1;
                working_rst <= '1';
                write_enable_s <= (others => '0');

            else
                finished <= '0';
                working_rst <= '0';
                current_step <= current_step + 1;
                if current_step >= 0 then
--                    write_enable_s <= (current_step => '1', others => '0');  -- Only enable the register for current step

                      write_enable_s <= to_STDLogicVector(write_enable_base_s sll current_step);
                else
                    write_enable_s <= (others => '0');
                end if;
            end if;
        else
            working_rst <= '0';                                              -- This is mandatory to prevent locking in reset state
        end if;
    end if;
end process;

-- Working is a memorised signal used to mark when the deserializer is currently working
-- Activated only when receiving an impulse from the sercom
working_memorisation : process(rst, main_clk, working_rst, des_ena)
begin
    if (rst = '1' or working_rst = '1') then
        working <= '0';
    end if;
    if(rising_edge(main_clk)) then
        if (des_ena = '1') then
            working <= '1';
        end if;
    end if;
end process;

end Behavioral;
