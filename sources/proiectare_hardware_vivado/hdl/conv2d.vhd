-- Caracteristici proiect:
--   * Nucleu KSIZE x KSIZE (KSIZE = 3 sau 5)
--   * Intrare/iesire de tip AXI4-Stream 
--   * BRAM (o singura memorie dual-port, latime 8*(KSIZE-1) biti)
--   * Fereastra KSIZE x KSIZE in registre (shift register)
--   * KSIZE*KSIZE multiplicatoare in paralel (DSP48) + arbore de sumatoare
--   * Pipeline pe 5 etaje -> 1 pixel / ciclu de ceas
--   * Convolutie "valid" (fara padding): iesire (W-K+1) x (H-K+1) pixeli
--   * Rezultat: (suma >> SHIFT) saturat in intervalul [0, 255]
--
-- Coeficienti: vector liniar, coeficientul (r,c) se afla la biturile
--   [8*(r*KSIZE+c)+7 : 8*(r*KSIZE+c)], signed pe 8 biti (-128..127)
--   r = rand (0 = sus), c = coloana (0 = stanga)
--
--   tot pipeline-ul avanseaza doar cand
--   consumatorul este gata (m_axis_tready = '1').
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity conv2d is
    generic (
        IMG_W : integer := 640;   -- latimea imaginii (pixeli)
        IMG_H : integer := 480;   -- inaltimea imaginii (pixeli)
        KSIZE : integer := 3      -- dimensiunea nucleului (3 sau 5)
    );
    port (
        clk           : in  std_logic;
        rst_n         : in  std_logic;  -- reset activ pe 0 

        coef          : in  std_logic_vector(KSIZE*KSIZE*8-1 downto 0);
        shift         : in  unsigned(4 downto 0);

        -- Stream slave (pixeli de intrare, rand cu rand)
        s_axis_tdata  : in  std_logic_vector(7 downto 0);
        s_axis_tvalid : in  std_logic;
        s_axis_tready : out std_logic;
        s_axis_tlast  : in  std_logic;  

        -- Stream master (pixeli de iesire)
        m_axis_tdata  : out std_logic_vector(7 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tready : in  std_logic;
        m_axis_tlast  : out std_logic
    );
end entity conv2d;

architecture rtl of conv2d is

    constant NTAPS   : integer := KSIZE * KSIZE;
    constant LB_W    : integer := 8 * (KSIZE - 1);   -- latimea cuvantului 
    constant ACC_W   : integer := 24;                 -- suficient pentru 25 * (255 * 128)

    type lb_ram_t  is array (0 to IMG_W-1) of std_logic_vector(LB_W-1 downto 0);
    type win_t     is array (0 to NTAPS-1) of unsigned(7 downto 0);  
    type prod_t    is array (0 to NTAPS-1) of signed(17 downto 0);
    type col_vec_t is array (0 to KSIZE-1) of unsigned(7 downto 0);

    signal en        : std_logic;                      
    signal in_fire   : std_logic;                       

    -- contoare de pozitie 
    signal col_cnt   : integer range 0 to IMG_W-1 := 0;
    signal row_cnt   : integer range 0 to IMG_H-1 := 0;

    -- Etaj 1: citire 
    signal lb_ram    : lb_ram_t := (others => (others => '0'));
    signal lb_rdata  : std_logic_vector(LB_W-1 downto 0) := (others => '0');
    signal pix_a     : unsigned(7 downto 0) := (others => '0');
    signal col_a     : integer range 0 to IMG_W-1 := 0;
    signal row_a     : integer range 0 to IMG_H-1 := 0;
    signal v_a       : std_logic := '0';

    -- Etaj 2: scriere + deplasare fereastra
    signal win       : win_t := (others => (others => '0'));
    signal v_b       : std_logic := '0';
    signal last_b    : std_logic := '0';

    -- Etaj 3: multiplicari
    signal prod      : prod_t := (others => (others => '0'));
    signal v_c       : std_logic := '0';
    signal last_c    : std_logic := '0';

    -- Etaj 4: sumare
    signal acc       : signed(ACC_W-1 downto 0) := (others => '0');
    signal v_d       : std_logic := '0';
    signal last_d    : std_logic := '0';

    -- Etaj 5: normalizare si saturare 
    signal out_data  : std_logic_vector(7 downto 0) := (others => '0');
    signal v_e       : std_logic := '0';
    signal last_e    : std_logic := '0';

    attribute ram_style : string;
    attribute ram_style of lb_ram : signal is "block";

begin
    en            <= m_axis_tready or (not v_e);
    s_axis_tready <= en;
    in_fire       <= s_axis_tvalid and en;

    p_counters : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                col_cnt <= 0;
                row_cnt <= 0;
            elsif in_fire = '1' then
                if col_cnt = IMG_W-1 then
                    col_cnt <= 0;
                    if row_cnt = IMG_H-1 then
                        row_cnt <= 0;
                    else
                        row_cnt <= row_cnt + 1;
                    end if;
                else
                    col_cnt <= col_cnt + 1;
                end if;
            end if;
        end if;
    end process;
    
    --Etajele 1 si 2
    p_line_buffer : process (clk)
    begin
        if rising_edge(clk) then
            if en = '1' then
                -- Etaj 1 
                lb_rdata <= lb_ram(col_cnt);
                pix_a    <= unsigned(s_axis_tdata);
                col_a    <= col_cnt;
                row_a    <= row_cnt;
                v_a      <= in_fire;

                -- Etaj 2 
                if v_a = '1' then
                    if KSIZE > 2 then
                        lb_ram(col_a) <= lb_rdata(LB_W-9 downto 0) & std_logic_vector(pix_a);
                    else
                        lb_ram(col_a) <= std_logic_vector(pix_a);
                    end if;
                end if;
            end if;
            if rst_n = '0' then
                v_a <= '0';
            end if;
        end if;
    end process;

    --Etaj 2
    p_window : process (clk)
        variable colv : col_vec_t;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_b    <= '0';
                last_b <= '0';
            elsif en = '1' then
                if v_a = '1' then
                    colv(KSIZE-1) := pix_a;
                    for j in 1 to KSIZE-1 loop
                        colv(KSIZE-1-j) := unsigned(lb_rdata(8*j-1 downto 8*(j-1)));
                    end loop;
                    for r in 0 to KSIZE-1 loop
                        for c in 0 to KSIZE-2 loop
                            win(r*KSIZE + c) <= win(r*KSIZE + c + 1);
                        end loop;
                        win(r*KSIZE + KSIZE-1) <= colv(r);
                    end loop;
                end if;

                if v_a = '1' and row_a >= KSIZE-1 and col_a >= KSIZE-1 then
                    v_b <= '1';
                else
                    v_b <= '0';
                end if;

                if v_a = '1' and row_a = IMG_H-1 and col_a = IMG_W-1 then
                    last_b <= '1';
                else
                    last_b <= '0';
                end if;
            end if;
        end if;
    end process;

    --Etaj 3
    p_mult : process (clk)
        variable k : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_c    <= '0';
                last_c <= '0';
            elsif en = '1' then
                for i in 0 to NTAPS-1 loop
                    k := signed(coef(8*i+7 downto 8*i));
                    prod(i) <= resize(signed('0' & win(i)) * k, 18);
                end loop;
                v_c    <= v_b;
                last_c <= last_b;
            end if;
        end if;
    end process;

    --Etaj 4
    p_sum : process (clk)
        variable s : signed(ACC_W-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_d    <= '0';
                last_d <= '0';
            elsif en = '1' then
                s := (others => '0');
                for i in 0 to NTAPS-1 loop
                    s := s + resize(prod(i), ACC_W);
                end loop;
                acc    <= s;
                v_d    <= v_c;
                last_d <= last_c;
            end if;
        end if;
    end process;

    --Etaj 5
    p_norm : process (clk)
        variable sh : signed(ACC_W-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_e    <= '0';
                last_e <= '0';
            elsif en = '1' then
                sh := shift_right(acc, to_integer(shift));
                if sh < 0 then
                    out_data <= (others => '0');
                elsif sh > 255 then
                    out_data <= (others => '1');
                else
                    out_data <= std_logic_vector(sh(7 downto 0));
                end if;
                v_e    <= v_d;
                last_e <= last_d;
            end if;
        end if;
    end process;

    m_axis_tdata  <= out_data;
    m_axis_tvalid <= v_e;
    m_axis_tlast  <= last_e;

end architecture rtl;
