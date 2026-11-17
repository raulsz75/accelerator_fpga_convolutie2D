library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_conv2d is
    generic (
        KSIZE : integer := 3
    );
end entity tb_conv2d;

architecture sim of tb_conv2d is

    constant IMG_W   : integer := 16;
    constant IMG_H   : integer := 12;
    constant OUT_W   : integer := IMG_W - KSIZE + 1;
    constant OUT_H   : integer := IMG_H - KSIZE + 1;
    constant NTAPS   : integer := KSIZE * KSIZE;
    constant NTESTS  : integer := 4;
    constant CLK_PER : time    := 10 ns;   -- 100 MHz

    function img_px(y, x : integer) return integer is
    begin
        return (y * 31 + x * 17 + ((x * y) mod 7) * 9) mod 256;
    end function;

    -- Nucleele de test
    --   1: identitate (doar centrul = 1)  -> iesirea = imaginea decupata
    --   2: blur (toti coeficientii = 1, shift = 3 pt 3x3 sau 5 pt 5x5)
    --   3: coeficienti cu semn -> testeaza saturarea
    --   4: Laplacian (centru pozitiv mare, vecini -1) -> valori negative -> 0
    
    function get_coef(t, i : integer) return integer is
        constant center : integer := (KSIZE / 2) * KSIZE + (KSIZE / 2);
    begin
        case t is
            when 1 =>
                if i = center then return 1; else return 0; end if;
            when 2 =>
                return 1;
            when 3 =>
                return ((i * 37 + 11) mod 16) - 8;
            when others =>
                if i = center then return NTAPS - 1; else return -1; end if;
        end case;
    end function;

    function get_shift(t : integer) return integer is
    begin
        case t is
            when 1      => return 0;
            when 2      => if KSIZE = 3 then return 3; else return 5; end if;
            when 3      => return 2;
            when others => return 0;
        end case;
    end function;

    function golden(t, oy, ox : integer) return integer is
        variable s : integer := 0;
        variable r : integer;
    begin
        for ky in 0 to KSIZE-1 loop
            for kx in 0 to KSIZE-1 loop
                s := s + img_px(oy + ky, ox + kx) * get_coef(t, ky * KSIZE + kx);
            end loop;
        end loop;
        r := to_integer(shift_right(to_signed(s, 32), get_shift(t)));
        if r < 0 then
            return 0;
        elsif r > 255 then
            return 255;
        else
            return r;
        end if;
    end function;

    signal clk      : std_logic := '0';
    signal rst_n    : std_logic := '0';
    signal sim_done : boolean   := false;

    signal coef     : std_logic_vector(NTAPS*8-1 downto 0) := (others => '0');
    signal shift    : unsigned(4 downto 0) := (others => '0');

    signal s_tdata  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_tvalid : std_logic := '0';
    signal s_tready : std_logic;
    signal s_tlast  : std_logic := '0';

    signal m_tdata  : std_logic_vector(7 downto 0);
    signal m_tvalid : std_logic;
    signal m_tready : std_logic := '0';
    signal m_tlast  : std_logic;

    signal test_done   : integer := 0;   
    signal error_count : integer := 0;

begin
    dut : entity work.conv2d
        generic map (
            IMG_W => IMG_W,
            IMG_H => IMG_H,
            KSIZE => KSIZE
        )
        port map (
            clk           => clk,
            rst_n         => rst_n,
            coef          => coef,
            shift         => shift,
            s_axis_tdata  => s_tdata,
            s_axis_tvalid => s_tvalid,
            s_axis_tready => s_tready,
            s_axis_tlast  => s_tlast,
            m_axis_tdata  => m_tdata,
            m_axis_tvalid => m_tvalid,
            m_axis_tready => m_tready,
            m_axis_tlast  => m_tlast
        );

    p_clk : process
    begin
        while not sim_done loop
            clk <= '0'; wait for CLK_PER / 2;
            clk <= '1'; wait for CLK_PER / 2;
        end loop;
        wait;
    end process;

    p_stim : process
        variable seed1 : positive := 11;
        variable seed2 : positive := 29;
        variable rnd   : real;
    begin
        rst_n <= '0';
        wait for 5 * CLK_PER;
        wait until rising_edge(clk);
        rst_n <= '1';

        for t in 1 to NTESTS loop
            for i in 0 to NTAPS-1 loop
                coef(8*i+7 downto 8*i) <= std_logic_vector(to_signed(get_coef(t, i), 8));
            end loop;
            shift <= to_unsigned(get_shift(t), 5);
            wait until rising_edge(clk);

            for y in 0 to IMG_H-1 loop
                for x in 0 to IMG_W-1 loop
                    uniform(seed1, seed2, rnd);
                    while rnd < 0.25 loop
                        s_tvalid <= '0';
                        wait until rising_edge(clk);
                        uniform(seed1, seed2, rnd);
                    end loop;

                    s_tdata  <= std_logic_vector(to_unsigned(img_px(y, x), 8));
                    s_tvalid <= '1';
                    if y = IMG_H-1 and x = IMG_W-1 then
                        s_tlast <= '1';
                    else
                        s_tlast <= '0';
                    end if;

                    loop
                        wait until rising_edge(clk);
                        exit when s_tready = '1';
                    end loop;
                end loop;
            end loop;
            s_tvalid <= '0';
            s_tlast  <= '0';

            wait until test_done = t;
        end loop;

        wait;
    end process;

    p_check : process
        variable seed1 : positive := 7;
        variable seed2 : positive := 3;
        variable rnd   : real;
        variable idx   : integer;
        variable oy, ox, exp_v, got_v : integer;
        variable errs  : integer := 0;
    begin
        wait until rst_n = '1';

        for t in 1 to NTESTS loop
            idx := 0;
            while idx < OUT_W * OUT_H loop
                wait until rising_edge(clk);

                if m_tvalid = '1' and m_tready = '1' then
                    oy    := idx / OUT_W;
                    ox    := idx mod OUT_W;
                    exp_v := golden(t, oy, ox);
                    got_v := to_integer(unsigned(m_tdata));

                    if got_v /= exp_v then
                        errs := errs + 1;
                        report "TEST " & integer'image(t) & " EROARE la (" &
                               integer'image(oy) & "," & integer'image(ox) &
                               "): asteptat " & integer'image(exp_v) &
                               ", primit " & integer'image(got_v)
                            severity error;
                    end if;

                    if (idx = OUT_W * OUT_H - 1) /= (m_tlast = '1') then
                        errs := errs + 1;
                        report "TEST " & integer'image(t) &
                               " EROARE tlast la indexul " & integer'image(idx)
                            severity error;
                    end if;

                    idx := idx + 1;
                end if;

                uniform(seed1, seed2, rnd);
                if rnd < 0.3 then
                    m_tready <= '0';
                else
                    m_tready <= '1';
                end if;
            end loop;

            report "TEST " & integer'image(t) & " terminat: " &
                   integer'image(OUT_W * OUT_H) & " pixeli verificati";
            error_count <= errs;
            test_done   <= t;
        end loop;

        wait for 10 * CLK_PER;
        if errs = 0 then
            report "===== SIMULARE REUSITA: toate testele au trecut (KSIZE = " &
                   integer'image(KSIZE) & ") =====";
        else
            report "===== SIMULARE ESUATA: " & integer'image(errs) &
                   " erori =====" severity error;
        end if;
        sim_done <= true;
        wait;
    end process;

    p_timeout : process
    begin
        wait for 1 ms;
        if not sim_done then
            report "TIMEOUT: simularea nu s-a terminat (posibil blocaj)"
                severity failure;
        end if;
        wait;
    end process;

end architecture sim;
