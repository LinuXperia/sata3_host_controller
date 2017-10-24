`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
//  Project     : SATA Host controller
//  Title       : Sata test
//  File name   : sata_test.v
//  Note        : This is module Sata test logic and interface between SATA 
//                controller and microblaze system
//  Dependencies   : 
///////////////////////////////////////////////////////////////////////////////

module SATA_WRAPPER(

    input             TILE0_REFCLK_PAD_P_IN,   // MGTCLKA,  clocks GTP_X0Y0-2 
    input             TILE0_REFCLK_PAD_N_IN,   // MGTCLKA 
    input             GTP_RESET_IN,            // GTP initialization
    output            TILE0_PLLLKDET_OUT,      // GTP PLL locked
    output            TXP0_OUT,                // SATA Connector TX P pin
    output            TXN0_OUT,                // SATA Connector TX N pin
    input             RXP0_IN,                 // SATA Connector RX P pin
    input             RXN0_IN,                 // SATA Connector RX N pin
    output            DCMLOCKED_OUT,           // PHY Layer DCM locked
    output            LINKUP,                  // SATA PHY initialisation completed LINK UP
    output     [1:0]  GEN,                     // 2 when SATA3, 1 when a SATA2 device detected, 0 when SATA1 device detected
 // output            PHY_CLK_OUT,             // PHY layer clock out
 // output            CLK_OUT,                 // LINK and Transport Layer clock out CLK_OUT = PHY_CLK_OUT / 2

    input      [56:0] ADDRESS_IN,
    input             WR_EN_IN,
    input             RD_EN_IN,
    input      [31:0] DATA_IN,
    output     [31:0] DATA_OUT,
    input             USR_CLOCK,
    input             USR_RESET,
    output            WR_HOLD_OUT,
    output reg        RD_HOLD_OUT,
    output reg        WR_DONE,
    input             OOB_reset_IN,
    input             RX_FSM_reset_IN,
    input             TX_FSM_reset_IN  
    

  );
    
  wire              CLK;
  wire              RESET;
  reg               CTRL_READ_EN;
  reg               CTRL_WRITE_EN;
  reg      [4 :0]   CTRL_ADDR_REG;
  reg      [31:0]   CTRL_DATA_OUT;
  wire     [31:0]   CTRL_DATA_IN;
  wire              SATA_WR_HOLD_IN;
  wire              SATA_RD_HOLD_IN;
  reg               DMA_RQST_OUT;
  wire     [31:0]   DMA_RX_DATA_IN;
  reg               DMA_RX_REN_OUT;
  wire     [31:0]   DMA_TX_DATA_OUT;
  reg               DMA_TX_WEN_OUT;
    
  wire              SATA_RESET_OUT;
  wire              INTERRUPT_IN;
  wire              DMA_TERMINATED;
  wire              R_ERR;
  wire              ILLEGAL_STATE;
  reg               RX_FIFO_RESET_OUT;
  reg               TX_FIFO_RESET_OUT;
 
  //for trasmition test logic
  //wire            tx_fifo_empty;
  //wire            tx_fifo_prog_empty;
  reg      [5 :0]   test_logic_state;
  //reg    [31:0]   tx_data;
  //reg             tx_data_wr_en;
  
  reg               test_button_down_int;
  reg      [31:0]   read_count;
  reg      [31:0]   sector_count;
  reg      [15:0]   counter_5us;
  reg      [31:0]   test_command_reg;
  reg      [31:0]   test_sector_count_reg;
  wire     [31:0]   test_status_reg;

  wire     [31:0]   test_lba_low_reg;
  wire     [31:0]   test_lba_mid_reg;
  wire     [31:0]   test_lba_high_reg;
  
  reg      [31:0]   throughput_count;
  reg               throughput_count_en;
  
  reg               cmd_en;
  reg               controller_reset;
  reg               error;
  reg               reset_controller;
  reg               reset_controller_1;
  reg               reset_controller_2;
  reg      [4 :0]   reset_count;
  reg               reset_counten;
  reg               tx_ram_rd_en;
  reg               rx_ram_wr_port_en;
  wire     [31:0]   rx_ram_wr_addr;
  wire     [31:0]   rx_ram_dout;
  reg               mb_cs_delayed;
  reg               mb_rnw_delayed;
  reg      [7 :0]   error_count;
  reg               error_from_SSD;
  reg      [31:0]   timeout_count;
  reg               timeout_count_en;
  reg               timeout_count_reset;
  reg               cmd_en_1;
  reg               cmd_en_2;


  reg      [1 :0]   ping_pong_state;
  reg      [47:0]   LBA1;
  reg      [47:0]   LBA2;
  reg      [47:0]   LBA;
  reg               write_hold_tx_ram1;
  reg               write_hold_tx_ram2;
  reg               ping_pong_tx_wr_en;
  reg               ping_pong_tx_rd_en;
  reg               read_hold_rx_ram1;
  reg               read_hold_rx_ram2;
  reg               ping_pong_rx_wr_en;
  reg               ping_pong_rx_rd_en ;
  reg               rd_en_in_delay;
  reg               write_operation;
  reg               first_read_operation;
  reg               cmd_complete_1;
  reg               cmd_complete_2;
  reg               rx1_empty;
  reg               rx2_empty;
  reg               tx1_empty;
  reg               tx2_empty;
  reg               tx1_full;
  reg               tx2_full;
  reg               read_operation;
  reg               first_write_complted;
  reg    [31:0]     ping_pong_timeout;
  reg               ping_pong_timeout_en;
  reg               user_finish_write;
  reg    [7:0]      sector_count_last_write;
  reg               user_finish_write_buffer;
  
  wire              sata_first_read_detect;
  reg               sata_first_read_detect_1;
  reg               sata_first_read_detect_2;
  reg               USR_RESET_1;
  reg               USR_RESET_2;
  reg               cmd_complete_count_en;
  reg    [2:0]      cmd_complete_count;
  reg               reset_linkup;
  reg               LINKUP_D;
  
  
  wire              rx_ram1_sel;
  reg               cmd_complete; 
  wire              rd_cmd_issue;
  //synthesis attribute keep of rd_cmd_issue is "true"
  wire              first_rd_cmd;
  wire    [31:0]    rx_ram_dout1;
  wire    [31:0]    rx_ram_dout2;
  wire    [31:0]    DMA_TX_DATA_OUT1;
  wire    [31:0]    DMA_TX_DATA_OUT2;
  wire              DMA_DATA_RCV_ERROR;
  
  
  
  reg               reset_once;
  
  parameter         data_reg               = 8'd11 ;

  /*
  parameter         wait_for_ROK_sent      = 4'b0000, 
                    read_cmd_reg           = 4'b0001,
                    read_ctrl_reg          = 4'b0010,
                    read_feature_reg       = 4'b0011, 
                    read_stuts_reg         = 4'b0100,
                    read_head_reg          = 4'b0101, 
                    read_error_reg         = 4'b0110,
                    read_lba_low           = 4'b0111,
                    read_lba_mid           = 4'b1000,
                    read_lba_high          = 4'b1001,
                    read_sect_count        = 4'b1010,
                    read_data_reg          = 4'b1011,
                    check_for_ROK_sent_low = 4'b1100; 
  */
  parameter         wait_for_BSY_0           = 6'h 0,
                    write_feature_reg        = 6'h 1, 
                    write_device_reg         = 6'h 2, 
                    write_LBA_low_reg        = 6'h 3, 
                    write_LBA_mid_reg        = 6'h 4, 
                    write_LBA_high_reg       = 6'h 5, 
                    write_sector_cnt_reg     = 6'h 6, 
                    write_cmd_reg            = 6'h 7, 
                    read_busy_bit            = 6'h 8,
                    check_for_BSY_1          = 6'h 9,    
                    
                    last_state               = 6'h 15,
                    read_data_reg_con        = 6'h 16,
                    write_ctrl_reg           = 6'h 17,
                    write_data_reg_con       = 6'h 18,
                    read_DMA                 = 6'h 19,
                    write_DMA                = 6'h 1A,
                    sector_count_check       = 6'h 1B,
                    
                    set_reset                     = 6'h A,
                    clear_reset                   = 6'h B,
                    wait_for_5us                  = 6'h C,
                    check_for_BSY_2               = 6'h D,
                    wait_for_cmd                  = 6'h E,
                    read_busy_bit_after_write_DMA = 6'h F,
                    check_for_BSY_3               = 6'h 10,
                    wait_for_linkup               = 6'h 11,
                    last_state_2                  = 6'h 12,
                    last_state_3                  = 6'h 13,
                    last_state_4                  = 6'h 14,
                    last_state_5                  = 6'h 1E,
                    last_state_6                  = 6'h 1F,
                    
                    read_busy_bit_1               = 6'h 1C,
                    read_DMA1                     = 6'h 1D;
                    

  parameter [31:0]  FIS_DW1        = 32'h 00EC8027, //32'h 00618027, //32'h01500027,
                    FIS_DW2        = 32'h A0000000, //32'h 08030201, //32'h00000001,
                    FIS_DW3        = 32'h 00000000, //32'h 08070605, //32'h00000000,
                    FIS_DW4        = 32'h 00000000, //32'h 0000000a, //32'h00000001,
                    FIS_DW5        = 32'h 00000000; //32'h 00000000; //32'h00000000;
                    
  parameter         R_W_IDENTIFICATION    = 2'b00;
  parameter         PING_PONG_WRITE       = 2'b01;
  parameter         PING_PONG_READ        = 2'b10;
  parameter         PING_PONG_READ1       = 2'b11;
  
  parameter         ADDR_WIDTH            = 16;
  parameter         SECTOR_COUNT          = 8'h80;  

  //sata reset generation
  //assign SATA_RESET_OUT = GTP_RESET_IN || test_reset_reg[0] || MB_RESET || controller_reset;
  //assign SATA_RESET_OUT = GTP_RESET_IN || test_reset_reg[0] || controller_reset;
  assign SATA_RESET_OUT = GTP_RESET_IN  || controller_reset;
   
  //assign CLK_OUT = CLK; 
   
  // Time out counter for writting data less than buffer  size
  always @(posedge USR_CLOCK, posedge USR_RESET)
  begin
    if (USR_RESET) begin    
      ping_pong_timeout   <= 32'h 0;
    end
    else begin
      if (ping_pong_timeout_en) begin
        ping_pong_timeout <= ping_pong_timeout + 1;
      end
      else begin
        ping_pong_timeout <= 32'b0;
      end
    end
  end  
   

  // Generating time out counter enable for writting  data less than buffer size
  always @(posedge USR_CLOCK, posedge USR_RESET)
  begin
    if (USR_RESET) begin    
      ping_pong_timeout_en <= 1'b0;
    end
    else begin
      if (ping_pong_state == PING_PONG_WRITE ) begin
        ping_pong_timeout_en <= 1'b1;
      end
      else begin
        ping_pong_timeout_en <= 1'b0;
      end
    end
  end 

  
  // Generating signal to indicate user stops write when the buffer not reaches its maximum
  always @(posedge USR_CLOCK, posedge USR_RESET)
  begin
    if (USR_RESET) begin    
      user_finish_write        <= 1'b0;
      user_finish_write_buffer <= 1'b0;
    end
    else begin
      if (ping_pong_timeout == 32'h00FFFFFF ) begin
        user_finish_write        <= 1'b1;
        user_finish_write_buffer <= ping_pong_tx_wr_en;
      end
      else if((user_finish_write_buffer == ping_pong_tx_rd_en) && sata_first_read_detect_2) begin
        user_finish_write        <= 1'b0;
        user_finish_write_buffer <= user_finish_write_buffer;
      end
      else begin
        user_finish_write        <= user_finish_write;
        user_finish_write_buffer <= user_finish_write_buffer;
      end
    end
  end 

  

  
  // Generating Write completion indicator signal
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      WR_DONE       <= 1'b0 ;
    end
    else if(first_write_complted && tx1_empty && tx2_empty)begin
      WR_DONE <= 1'b1;
    end
   else if(((ping_pong_state == R_W_IDENTIFICATION) || (ping_pong_state == PING_PONG_WRITE))  && WR_EN_IN) begin
     WR_DONE  <= 1'b0;
   end
    else begin
      WR_DONE       <= WR_DONE ;
    end
  end
  
  // Generating Write buffer empty  signals
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      tx1_empty       <= 1'b1;
      tx2_empty       <= 1'b1;
    end
    else if( (ping_pong_state == R_W_IDENTIFICATION) && WR_EN_IN)begin
      if(ping_pong_tx_wr_en) begin
        tx1_empty       <= 1'b0;
      end
     else begin
       tx2_empty        <= 1'b0;
      end
    end
    else if(write_operation && cmd_complete_2 && cmd_en)begin
      if(ping_pong_tx_rd_en) begin
        tx1_empty       <= 1'b1;
      end
      else begin
        tx2_empty         <= 1'b1;
      end
    end
    else begin
      tx1_empty       <= tx1_empty;
      tx2_empty       <= tx2_empty;
    end
  end
  
  // Generating first_write_complted signals
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      first_write_complted <= 1'b0;
    end
    else if(write_operation && cmd_complete_2 && cmd_en)begin
      first_write_complted <= 1'b1;
    end
    else begin
      first_write_complted <= first_write_complted;
    end
  end
  
  
  assign WR_HOLD_OUT = write_hold_tx_ram1 & write_hold_tx_ram2;

  
  //ping_pong fifo handling state machine
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      ping_pong_state         <= R_W_IDENTIFICATION ; 
      LBA1                    <= 48'b0;
      LBA2                    <= 48'b0;
      ping_pong_tx_wr_en      <= 1'b1;
      ping_pong_rx_rd_en      <= 1'b1;
      rd_en_in_delay          <= 1'b0;
      read_operation          <= 1'b0;
      sector_count_last_write <= 8'h0;
    end
    else begin
      case (ping_pong_state)
       
        R_W_IDENTIFICATION: begin
          if (WR_EN_IN) begin
            ping_pong_state      <= PING_PONG_WRITE;
            if(ping_pong_tx_wr_en) begin
              LBA1                 <= ADDRESS_IN[56:9];
            end
            else begin
              LBA2                 <= ADDRESS_IN[56:9];
            end
          end
          else if(RD_EN_IN) begin
            ping_pong_state    <= PING_PONG_READ;
            LBA1               <= ADDRESS_IN[56:9];
            read_operation     <= 1'b1;
          end
          else begin
            ping_pong_state    <= ping_pong_state;
          end
        end 
  
        PING_PONG_WRITE: begin
          if(WR_EN_IN) begin
            if(ADDRESS_IN[(ADDR_WIDTH-1):2] == 14'h 3fff ) begin
              ping_pong_state    <= R_W_IDENTIFICATION;
              ping_pong_tx_wr_en <= !ping_pong_tx_wr_en;
            end
            else if(user_finish_write) begin
              ping_pong_state    <= R_W_IDENTIFICATION;
              ping_pong_tx_wr_en <= !ping_pong_tx_wr_en;
            end
            else begin
              ping_pong_state <= ping_pong_state;
            end 
          sector_count_last_write <= ADDRESS_IN[15:9] + 1'b1;
          end
        end


        
        PING_PONG_READ: begin
          if(RD_EN_IN && (ADDRESS_IN[(ADDR_WIDTH-1):2] == 14'h 3fff)) begin
            ping_pong_state    <= PING_PONG_READ1;
          end
          else begin
            ping_pong_state <= PING_PONG_READ;
          end
        end
 
        PING_PONG_READ1: begin
          ping_pong_rx_rd_en <= !ping_pong_rx_rd_en;
          ping_pong_state    <= PING_PONG_READ;
        end
      endcase
    end
  end 
  
  
 // generating HOLD for ping & pong  TX buffers
  always @(posedge USR_CLOCK, posedge USR_RESET)  begin
    if (USR_RESET) begin
      write_hold_tx_ram1 <= 1'b0;
      write_hold_tx_ram2 <= 1'b0;
    end
    else if((ping_pong_state == PING_PONG_WRITE) && WR_EN_IN && (ADDRESS_IN[(ADDR_WIDTH-1):2] == 14'h 3fff )) begin
      if(ping_pong_tx_wr_en) begin 
        write_hold_tx_ram1 <= 1'b1;
      end
      else begin
        write_hold_tx_ram2 <= 1'b1;
      end
    end
    else if (write_operation && cmd_complete_2 && cmd_en) begin
      if(ping_pong_tx_rd_en) begin 
        write_hold_tx_ram1 <= 1'b0;
      end
      else begin
        write_hold_tx_ram2 <= 1'b0;
      end
    end
    else if(ping_pong_state == PING_PONG_READ) begin
      write_hold_tx_ram1 <= 1'b1;
      write_hold_tx_ram2 <= 1'b1;
    end
    else begin
      write_hold_tx_ram1 <= write_hold_tx_ram1;
      write_hold_tx_ram2 <= write_hold_tx_ram2;
    end
  end
  
  assign read_hold_rx_ram = read_hold_rx_ram1 & read_hold_rx_ram2;
  
  
 
   // generating HOLD for ping & pong  RX buffers
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      read_hold_rx_ram1    <= 1'b1;
      read_hold_rx_ram2    <= 1'b1;
    end
    else if (read_operation && cmd_complete_2 && cmd_en) begin
      if(ping_pong_rx_wr_en) begin 
        read_hold_rx_ram1  <= 1'b0;
      end
      else begin
        read_hold_rx_ram2  <= 1'b0;
      end
    end
    else if( RD_EN_IN && (ADDRESS_IN[(ADDR_WIDTH-1):2] == 14'h 3fff))begin
      if(ping_pong_rx_rd_en) begin
        read_hold_rx_ram1 <= 1'b1;
      end
      else begin
        read_hold_rx_ram2 <= 1'b1;
      end
    end
    else begin
      read_hold_rx_ram1    <= read_hold_rx_ram1;
      read_hold_rx_ram2    <= read_hold_rx_ram2;
    end
  end
  
   //generating RD_HOLD_OUT signal
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      RD_HOLD_OUT         <= 1'b0;
    end
    else if((ping_pong_state == R_W_IDENTIFICATION) && RD_EN_IN ) begin
      RD_HOLD_OUT     <= 1'b1;
    end
    else if((ping_pong_state == PING_PONG_READ)||(ping_pong_state == PING_PONG_READ1)) begin
      if (read_operation && cmd_complete_2 && cmd_en) begin
        RD_HOLD_OUT     <= 1'b0;
      end
      else if( RD_EN_IN && (ADDRESS_IN[(ADDR_WIDTH-1):2] == 14'h 3fff)) begin
        RD_HOLD_OUT     <= 1'b1;
      end
      else begin
        RD_HOLD_OUT     <= RD_HOLD_OUT;
      end
    end
    else if (ping_pong_state == PING_PONG_WRITE) begin
      RD_HOLD_OUT     <= 1'b1;
    end
    else begin
      RD_HOLD_OUT          <= RD_HOLD_OUT;
    end
  end
   
   //generating rx buffer empty signals
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if(USR_RESET) begin
      rx1_empty <= 1'b1;
      rx2_empty <= 1'b1;
    end
    else begin
      if( RD_EN_IN && (ADDRESS_IN[(ADDR_WIDTH-1):2] == 14'h 3fff ))begin
        if(ping_pong_rx_rd_en) begin
          rx1_empty <= 1'b1;
        end
        else begin
          rx2_empty <= 1'b1;
        end
      end
      if((rx_ram_wr_addr[(ADDR_WIDTH-1):5]== 11'h0) && DMA_RX_REN_OUT )
      begin
        if(rx_ram1_sel) begin
          rx1_empty <= 1'b0;
        end
        else begin
          rx2_empty <= 1'b0;
        end
      end
    end
  end
  
 
 assign rd_cmd_issue = (ping_pong_state == PING_PONG_READ) ? (rx1_empty | rx2_empty) : 1'b0 ;  
 assign first_rd_cmd = (ping_pong_state == R_W_IDENTIFICATION) ? RD_EN_IN : 1'b0;  
 
  // checking first read operation
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      first_read_operation <= 1'b0;
    end
    else if((ping_pong_state == R_W_IDENTIFICATION) && RD_EN_IN) begin
      first_read_operation <= 1'b1;
    end
    else if (read_operation && cmd_complete_2 && first_read_operation) begin
      first_read_operation <= 1'b0;
    end
  end
  
   //generating tx buffer full signals
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if(USR_RESET) begin
      tx1_full <= 1'b0;
      tx2_full <= 1'b0;
    end
    else if(ping_pong_state == PING_PONG_READ) begin
      tx1_full <= 1'b0;
      tx2_full <= 1'b0;
    end
    else begin
      if( (ping_pong_state == PING_PONG_WRITE)&& WR_EN_IN &&(ADDRESS_IN[(ADDR_WIDTH-1):2] == 14'h 3fff ) )begin
        if(ping_pong_tx_wr_en) begin
          tx1_full <= 1'b1;
        end
        else begin
          tx2_full <= 1'b1;
        end
      end
      if(sata_first_read_detect_2)
      begin
        if(ping_pong_tx_rd_en) begin
          tx1_full <= 1'b0;
        end
        else begin
          tx2_full <= 1'b0;
        end
      end
    end
  end
  
  assign sata_first_read_detect = ((read_count[(ADDR_WIDTH-1):2] >= 14'h1) && (read_count[(ADDR_WIDTH-1):2] <= 14'h8)) ? 1'b1 : 1'b0;
  
  //synchronising sata_first_read_detect
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      sata_first_read_detect_1 <= 0;
      sata_first_read_detect_2 <= 0;
    end
    else begin
      sata_first_read_detect_1 <= sata_first_read_detect;
      sata_first_read_detect_2 <= sata_first_read_detect_1;
    end
  end

  //generating cmd_en to start a sata command
  always @(posedge USR_CLOCK, posedge USR_RESET) begin
    if (USR_RESET) begin
      cmd_en                 <= 1'b0;
      test_command_reg       <= 32'h00000000;
      test_sector_count_reg  <= 32'b0;
      LBA                    <= 48'b0;
      ping_pong_tx_rd_en     <= 1'b1;
      ping_pong_rx_wr_en     <= 1'b1;
      write_operation        <= 1'b0;
    end
    else begin
      if ((tx1_full || tx2_full || user_finish_write) && ((cmd_en == 0) && 
               (cmd_complete_count_en == 0))) begin
        cmd_en                 <= 1'b1;
        write_operation        <= 1'b1;
        test_command_reg       <= 32'h00000002;
        if(ping_pong_tx_rd_en)begin
          LBA                    <= LBA1;
        end
        else begin
          LBA                    <= LBA2;
        end
        if(user_finish_write)begin
          test_sector_count_reg  <= {24'b0,sector_count_last_write};
        end
        else begin
          test_sector_count_reg  <= {24'b0,SECTOR_COUNT};
        end
      end 
      else if( rd_cmd_issue && (cmd_en == 0) && (cmd_complete_count_en == 0)) begin
        cmd_en                <= 1'b1;
        write_operation       <= 1'b0;
        test_command_reg      <= 32'h00000001;
        test_sector_count_reg <= {24'b0,SECTOR_COUNT}; 
        if(first_read_operation) begin
          LBA                   <= LBA1;
        end
        else begin
          LBA                   <= LBA + SECTOR_COUNT;
        end
      end
      else if (read_operation && cmd_complete_2 && cmd_en) begin
        ping_pong_rx_wr_en <= !ping_pong_rx_wr_en;
        cmd_en             <= 1'b0;
        write_operation    <= 1'b0;
      end
      else if (write_operation && cmd_complete_2 && cmd_en) begin
        cmd_en             <= 1'b0;
        write_operation    <= 1'b0;
        ping_pong_tx_rd_en <= !ping_pong_tx_rd_en;
      end
      else begin
        cmd_en                <= cmd_en;
        LBA                   <= LBA;
        test_command_reg      <= test_command_reg;
        test_sector_count_reg <= test_sector_count_reg;
        write_operation       <= write_operation;
      end
    end
  end
            

  //synchronising cmd_en
  always @(posedge CLK, posedge RESET)
  begin
    if (RESET) begin
      cmd_en_1 <= 0;
      cmd_en_2 <= 0;
    end
    else begin
      cmd_en_1 <= cmd_en;
      cmd_en_2 <= cmd_en_1;
    end
  end 
  
  //generating cmd_complete
  always @(posedge CLK, posedge RESET)
  begin
    if (RESET) begin
      cmd_complete <= 0;
    end else begin
      if ((((test_logic_state == last_state)  || 
            (test_logic_state == last_state_2)  ) && !error && !error_from_SSD) || (error_count >= 8'h1F)) begin
        cmd_complete <= 1;
      end else begin
        cmd_complete <= 0;
      end
    end
  end 
  
  //assign cmd_complete = (((test_logic_state == last_state) && !error && !error_from_SSD) || (error_count >= 8'h1F)) ? 1'b1:1'b0; 

  //Below two process are used to insert delay between two consecutive cmd_en
  //This delay is required inorder to reset error_count.
  always @(posedge USR_CLOCK, posedge USR_RESET)
  begin
    if (USR_RESET) begin
      cmd_complete_count_en <= 1'b0;
    end 
    else if (cmd_complete_2) begin
      cmd_complete_count_en <= 1'b1;
    end
    else if(cmd_complete_count == 3'b101) begin
      cmd_complete_count_en <= 1'b0;
    end
    else begin
      cmd_complete_count_en <= cmd_complete_count_en;
    end
  end
  
  always @(posedge USR_CLOCK, posedge USR_RESET)
  begin
    if (USR_RESET) begin
      cmd_complete_count <= 3'b000;
    end 
    else if (cmd_complete_count_en) begin
      cmd_complete_count <= cmd_complete_count +1;
    end
    else begin
      cmd_complete_count <= 3'b000;
    end
  end
  
  //synchronising cmd_complete
  always @(posedge USR_CLOCK, posedge USR_RESET)
  begin
    if (USR_RESET) begin
      cmd_complete_1 <= 0;
      cmd_complete_2 <= 0;
    end
    else begin
      cmd_complete_1 <= cmd_complete;
      cmd_complete_2 <= cmd_complete_1;
    end
  end 

  assign test_status_reg[0] = cmd_en;
  assign test_status_reg[1] = error;
  assign test_status_reg[2] = error_from_SSD;
  
   
 //LBA address to HDD LBA register mapping
  assign test_lba_low_reg[7:0]   = LBA[7:0];
  assign test_lba_mid_reg[7:0]   = LBA[15:8];
  assign test_lba_high_reg[7:0]  = LBA[23:16];
  assign test_lba_low_reg[15:8]  = LBA[31:24];
  assign test_lba_mid_reg[15:8]  = LBA[39:32];
  assign test_lba_high_reg[15:8] = LBA[47:40];
  
   
//throughput counter process
  always @(posedge CLK, posedge RESET)
  begin
    if (RESET) begin    
      throughput_count <= 32'h 0;
    end
    else begin
      if (test_logic_state ==  write_cmd_reg) begin
        throughput_count <=  32'h 0;
      end
      else if (throughput_count_en) begin
        throughput_count <= throughput_count + 1;
      end
      else begin
        throughput_count <= throughput_count;
      end
    end
  end
  
  //timeout-count counter process
  always @(posedge CLK, posedge RESET)
  begin
    if (RESET) begin    
      timeout_count <= 32'h 0;
    end
    else begin
      if (timeout_count_reset ==  1) begin
        timeout_count <=  32'h 0;
      end
      else if (timeout_count_en) begin
        timeout_count <= timeout_count + 1;
      end
      else begin
        timeout_count <= timeout_count;
      end
    end
  end  
  
  //Synchronising reset_controller
  always @(posedge USR_CLOCK, posedge USR_RESET)
  begin
    if (USR_RESET) begin
      reset_controller_1 <= 1'b0;
      reset_controller_2 <= 1'b0;
    end
    else begin
      reset_controller_1 <= reset_controller;
      reset_controller_2 <= reset_controller_1;
    end
  end
  
  //resetting controller
  always @(posedge USR_CLOCK, posedge USR_RESET)
  begin
    if (USR_RESET) begin
      controller_reset <= 0;
      reset_count      <= 5'h 0;
      reset_counten    <= 0;
    end
    else begin
    
      if (reset_controller_2) begin
        reset_counten <= 1;
        controller_reset <= 0;
      end
      else if (reset_count == 5'h 8) begin
        controller_reset <= 1;
        reset_counten    <= 1;
      end
      else if (reset_count == 5'h 1F) begin
        controller_reset <= 0;
        reset_counten    <= 0;
      end
      else begin
        reset_counten    <= reset_counten;
        controller_reset <= controller_reset;
      end
      
      
      //count for controller reset reset
      if (reset_counten) begin
        reset_count <= reset_count + 1;
      end
      else begin
        reset_count <= 5'd0;
      end
      
    end
  end
  
  //assign controller_reset = ((reset_count >= 4'h 5) && (reset_count <= 4'h F))? 1 : 0;
  
  //synchronising user reset (USR_RESET)
  always @(posedge CLK)
  begin
    USR_RESET_1 <= USR_RESET; 
    USR_RESET_2 <= USR_RESET_1; 
  end
  
  //synchronising user reset (USR_RESET)
//  always @(posedge CLK)
//  begin
//    cmd_complete_count_en_1 <= cmd_complete_count_en; 
//    cmd_complete_count_en_2 <= cmd_complete_count_en_1; 
//  end

  //Delaying LINKUP by one clock
  always @(posedge CLK, posedge USR_RESET_2)
  begin
    if (USR_RESET_2) begin
      LINKUP_D <= 1'b0;
    end
    else begin
      LINKUP_D <= LINKUP;
    end
  end 


  //Generating reset_linkup for compensating LINKUP not become low just after GTPRESET
  always @(posedge CLK, posedge USR_RESET_2)
  begin
    if (USR_RESET_2) begin
      reset_linkup <= 1'b0;
    end
    else if(reset_controller) begin
      reset_linkup <= 1'b1;
    end
    else if(LINKUP && (!LINKUP_D)) begin
      reset_linkup <= 1'b0;
    end
    else begin
      reset_linkup <= reset_linkup;
    end
  end
  
  
      
  //main test state machine.
  always @(posedge CLK, posedge USR_RESET_2)
  begin
    if (USR_RESET_2) begin  
      CTRL_WRITE_EN       <= 0;
      CTRL_READ_EN        <= 0;
      CTRL_ADDR_REG       <= 5'h 0;
      test_logic_state    <= wait_for_cmd;
      read_count          <= 32'h0;
      DMA_RQST_OUT        <= 0;
      sector_count        <= 32'h0;
      throughput_count_en <= 0;
      reset_controller    <= 0;
      error               <= 0;
      tx_ram_rd_en        <= 0;
      rx_ram_wr_port_en   <= 0;
      RX_FIFO_RESET_OUT   <= 0;
      TX_FIFO_RESET_OUT   <= 0;
      error_count         <= 8'h 0;
      error_from_SSD      <= 0;
      timeout_count_en    <= 0;
      timeout_count_reset <= 1;
      counter_5us         <= 16'h 0;
      CTRL_DATA_OUT       <= 32'h 0;
      DMA_RX_REN_OUT      <= 0;
      DMA_TX_WEN_OUT      <= 0;
      reset_once          <= 1;
    end
    else begin
      case (test_logic_state)

        wait_for_cmd: begin
          if (cmd_en_2 && !cmd_complete_count_en) begin
            test_logic_state    <= wait_for_linkup;
            timeout_count_en    <= 1;
            timeout_count_reset <= 1;
            error               <= 0;
          end
          else begin
            test_logic_state    <= wait_for_cmd;
            error_count         <= 8'h 0;
            timeout_count_reset <= 0;
          end
        end 
        
        wait_for_linkup: begin
          timeout_count_reset <= 0;
          if(reset_linkup) begin
            test_logic_state <= wait_for_linkup;
          end    
          else if (LINKUP) begin //&& !controller_reset
            if (reset_once) begin
              test_logic_state <= set_reset;
              counter_5us      <= 16'hFFFF;
            end
            else begin
              test_logic_state <= read_busy_bit_1;
            end
            RX_FIFO_RESET_OUT <= 1;
            TX_FIFO_RESET_OUT <= 1;
          end
          else if ((timeout_count == 32'h 000FFFFF)) begin
              test_logic_state <= last_state;
              timeout_count_en <= 0;
              reset_controller <= 1;
              error            <= 1;
          end
          else if ((timeout_count == 32'h 000FFFFE)) begin
              test_logic_state <= wait_for_linkup;
              reset_controller <= 1;
              error            <= 1;
          end
          else if ((timeout_count == 32'h 000FFFFD)) begin
              test_logic_state <= wait_for_linkup;
              reset_controller <= 1;
              error            <= 1;
              error_count      <= error_count + 1;
              reset_once       <= 1;
          end
          else begin
            test_logic_state <= wait_for_linkup;
          end
        end
        
        set_reset: begin                          //device reset 
          CTRL_READ_EN      <= 1'b 0;             
          CTRL_WRITE_EN     <= 1'b 1;             
          CTRL_ADDR_REG     <= 5'h 2;             //control register
          CTRL_DATA_OUT      <= {24'b 0,8'h 04};   
          test_logic_state  <= wait_for_5us;
          counter_5us       <= 16'd376;  //d'752 @SATA3 (150MHz) //'d376;   @SATA2 (75Mhz)   //5us
          error             <= 0;
          reset_once        <= 0;
        end
                  
        wait_for_5us: begin
          counter_5us <= counter_5us -1;
          CTRL_READ_EN      <= 1'b 0;                                           
          CTRL_WRITE_EN     <= 1'b 0; 
          if (counter_5us == 16'd0) begin
            test_logic_state  <= clear_reset;
          end
          else begin
            test_logic_state  <= wait_for_5us;
          end
        end
        
        clear_reset: begin
          CTRL_READ_EN        <= 1'b 0;
          CTRL_WRITE_EN       <= 1'b 1;
          CTRL_ADDR_REG       <= 5'h 2;             //control register
          CTRL_DATA_OUT       <= {24'b 0,8'h 00};
          test_logic_state    <= read_busy_bit_1;
          //counter_5us       <= 16'hFFFF;
          timeout_count_en    <= 1;
          timeout_count_reset <= 1;
        end
        
        read_busy_bit_1: begin
          //timeout_count_reset  <= 0;
          CTRL_READ_EN      <= 1'b 1;              
          CTRL_WRITE_EN     <= 1'b 0;              
          CTRL_ADDR_REG     <= 5'h 4;
          test_logic_state  <= wait_for_BSY_0;
			 timeout_count_en    <= 1;
          timeout_count_reset <= 1;

        end        
          
        wait_for_BSY_0: begin
          timeout_count_reset <= 0;
          CTRL_READ_EN        <= 1'b 1;
          CTRL_WRITE_EN       <= 1'b 0;      
          CTRL_ADDR_REG       <= 5'h 4;
          RX_FIFO_RESET_OUT   <= 0;
          TX_FIFO_RESET_OUT   <= 0;
          if (timeout_count == 32'h 001FFFFF) begin
            test_logic_state  <= last_state;
            reset_controller  <= 1;
            timeout_count_en  <= 0;
            error             <= 1;
          end
          else if (timeout_count == 32'h 001FFFFE) begin
            test_logic_state  <= wait_for_BSY_0;
            reset_controller  <= 1;
            error             <= 1;
          end
          else if (timeout_count == 32'h 001FFFFD) begin
            test_logic_state  <= wait_for_BSY_0;
            reset_controller  <= 1;
            error             <= 1;
            error_count       <= error_count + 1;
            reset_once        <= 1;
          end
          else if(CTRL_DATA_IN[7] == 1'b0)begin
            test_logic_state <= write_feature_reg;
            timeout_count_en <= 0;
          end
          else begin
            test_logic_state <= wait_for_BSY_0; 
          end            
        end
        
//        check_for_BSY_2: begin
//          CTRL_READ_EN      <= 1'b 1;              
//          CTRL_WRITE_EN     <= 1'b 0;              
//          CTRL_ADDR_REG     <= 5'h 4;
//          if(CTRL_DATA_IN[7] == 1'b1)begin
//            test_logic_state <= check_for_BSY_2;
//          end
//          else begin
//            test_logic_state <= write_feature_reg;
//          end        
//        end

        write_feature_reg: begin                                                
          CTRL_READ_EN      <= 1'b 0;                                           
          CTRL_WRITE_EN     <= 1'b 1;                                           
          CTRL_ADDR_REG     <= 5'h 3;                                           
          CTRL_DATA_OUT     <= 32'h0;                                          
          test_logic_state  <= write_device_reg;                                
        end                                                                     
        write_device_reg: begin                                                 
          CTRL_READ_EN      <= 1'b 0;                                           
          CTRL_WRITE_EN     <= 1'b 1;                                           
          CTRL_ADDR_REG     <= 5'h 5;               
          CTRL_DATA_OUT     <= {24'b 0,8'h E0};   
          test_logic_state  <= write_LBA_low_reg;
        end        
        write_LBA_low_reg: begin
          CTRL_READ_EN      <= 1'b 0;              
          CTRL_WRITE_EN     <= 1'b 1;              
          CTRL_ADDR_REG     <= 5'h 7;               
          CTRL_DATA_OUT     <= test_lba_low_reg; //32'h 0;    
          test_logic_state  <= write_LBA_mid_reg;
        end
        write_LBA_mid_reg: begin
          CTRL_READ_EN      <= 1'b 0;               
          CTRL_WRITE_EN     <= 1'b 1;               
          CTRL_ADDR_REG     <= 5'h 8;               
          CTRL_DATA_OUT     <= test_lba_mid_reg; //32'h 0;             
          test_logic_state  <= write_LBA_high_reg;        
        end
        write_LBA_high_reg:begin
           CTRL_READ_EN      <= 1'b 0;              
           CTRL_WRITE_EN     <= 1'b 1;              
           CTRL_ADDR_REG     <= 5'h 9;              
           CTRL_DATA_OUT     <= test_lba_high_reg; //32'h 0;            
           test_logic_state  <= write_sector_cnt_reg;   
        end
        write_sector_cnt_reg: begin
           CTRL_READ_EN      <= 1'b 0;
           CTRL_WRITE_EN     <= 1'b 1;
           CTRL_ADDR_REG     <= 5'h A;
           CTRL_DATA_OUT     <= test_sector_count_reg;           //no. of sectors
           sector_count      <= test_sector_count_reg;
           test_logic_state  <= write_cmd_reg; 
        end
        write_cmd_reg: begin
          CTRL_READ_EN      <= 1'b 0;
          CTRL_ADDR_REG     <= 5'h 1;
          error_from_SSD    <= 0;
          error             <= 0;
          
          //CTRL_DATA_OUT      <= {24'b0,8'hEC}; test_logic_state  <= read_busy_bit; //Identify Device
          //CTRL_DATA_OUT      <= {24'b0,8'h24}; test_logic_state  <= read_busy_bit; //PIO read
          //CTRL_DATA_OUT      <= {24'b0,8'h34}; test_logic_state  <= read_busy_bit; //PIO write
          if (test_command_reg == 32'h1) begin
            CTRL_DATA_OUT       <= {24'b0,8'h25}; 
            DMA_RQST_OUT        <= 1; 
            test_logic_state    <= read_DMA;//DMA read
            throughput_count_en <= 1;
            CTRL_WRITE_EN       <= 1'b 1;  
          end
          else if (test_command_reg == 32'h2) begin
            CTRL_DATA_OUT       <= {24'b0,8'h35}; 
            //CTRL_DATA_OUT      <= {24'b0,8'hCA}; 
            DMA_RQST_OUT <= 1; 
            test_logic_state <= write_DMA;//DMA write
            throughput_count_en <= 1;
            CTRL_WRITE_EN       <= 1'b 1; 
            tx_ram_rd_en        <= 1'b 1;
          end
          else begin
            CTRL_WRITE_EN       <= 1'b 0; 
            test_logic_state    <= last_state;
            throughput_count_en <= 0;
          end
        end
        //PIO rcv data
        read_busy_bit: begin
          CTRL_READ_EN      <= 1'b 1;              
          CTRL_WRITE_EN     <= 1'b 0;              
          CTRL_ADDR_REG     <= 5'h 4;
          test_logic_state  <= check_for_BSY_1;
        end
        check_for_BSY_1: begin
          CTRL_READ_EN      <= 1'b 1;              
          CTRL_WRITE_EN     <= 1'b 0;              
          CTRL_ADDR_REG     <= 5'h 4;
          if(CTRL_DATA_IN[7] == 1'b1)begin
            test_logic_state <= check_for_BSY_1;
          end
          else begin
            read_count       <= 32'h0;
            CTRL_READ_EN     <= 1'b 0;
            CTRL_WRITE_EN    <= 1'b 0;
            //test_logic_state <= read_data_reg_con;
            test_logic_state <= write_data_reg_con;
            
          end        
        end 
        
        write_data_reg_con: begin
          if (SATA_WR_HOLD_IN)begin
            CTRL_WRITE_EN     <= 0;
            CTRL_ADDR_REG    <= data_reg;
            test_logic_state <= write_data_reg_con;
            read_count       <= read_count;
            CTRL_DATA_OUT    <= CTRL_DATA_OUT;
          end
          else begin
            if (read_count < 32'h200) begin
              test_logic_state <= write_data_reg_con;
              CTRL_WRITE_EN     <= 1;
              CTRL_ADDR_REG    <= data_reg;
              read_count       <= read_count + 4;
              CTRL_DATA_OUT    <= read_count;
            end
            else begin
              test_logic_state <= sector_count_check;
              sector_count     <= sector_count - 1;
              CTRL_READ_EN     <= 1'b 1;
              CTRL_WRITE_EN    <= 0;
              CTRL_ADDR_REG    <= 5'h 4;
              read_count       <= 32'h0;
              CTRL_DATA_OUT    <= 32'h0;
            end
          end
        end
        
        sector_count_check: begin
          if (sector_count == 0) begin
            test_logic_state <= last_state;
          end
          else begin
            test_logic_state <= read_busy_bit;
          end
        end

        read_data_reg_con: begin
          if (SATA_RD_HOLD_IN)begin
            CTRL_READ_EN     <= 0;
            CTRL_ADDR_REG    <= data_reg;
            test_logic_state <= read_data_reg_con;
            read_count       <= read_count;
          end
          else begin
            if (read_count < {test_sector_count_reg << 1, 8'h00}) begin
              test_logic_state <= read_data_reg_con;
              CTRL_READ_EN     <= 1;
              CTRL_ADDR_REG    <= data_reg;
              read_count       <= read_count + 4;
            end
            else begin
              test_logic_state <= last_state;
              CTRL_READ_EN     <= 0;
              CTRL_ADDR_REG    <= data_reg;
              read_count       <= 32'h0;
            end
          end
        end
        
        read_DMA: begin
          //if (R_ERR || ILLEGAL_STATE) begin
          if (DMA_DATA_RCV_ERROR) begin
              test_logic_state <= last_state;
              CTRL_READ_EN     <= 0;
              CTRL_WRITE_EN    <= 0;
              DMA_RX_REN_OUT   <= 0;
              read_count       <= 32'h0;
              //DMA_TX_DATA_OUT  <= 32'h0;
              error            <= 1;
              error_count      <= error_count + 1;
              reset_once       <= 1;
          end
          else if (SATA_RD_HOLD_IN) begin
            DMA_RX_REN_OUT   <= 0;
            CTRL_READ_EN     <= 0;
            CTRL_WRITE_EN    <= 0;
            
            if ((read_count >= ({test_sector_count_reg << 1, 8'h00}))) begin
              test_logic_state <= read_busy_bit_after_write_DMA;
            end
            else if ((throughput_count == 32'h 00FFFFFF)) begin
              test_logic_state <= last_state;
              error            <= 1;
              error_count      <= error_count + 1;
              reset_once       <= 1;
            end
            else begin
              test_logic_state <= read_DMA;
            end
            
            read_count       <= read_count;
          end
          else begin
            test_logic_state <= read_DMA1;
            DMA_RX_REN_OUT    <= 1;
            CTRL_READ_EN      <= 0;
            CTRL_WRITE_EN     <= 0;
            read_count        <= read_count;
            rx_ram_wr_port_en <= 1;
          end
        end
        read_DMA1: begin
          if (DMA_DATA_RCV_ERROR) begin
              test_logic_state <= last_state;
              CTRL_READ_EN     <= 0;
              CTRL_WRITE_EN    <= 0;
              DMA_RX_REN_OUT   <= 0;
              read_count       <= 32'h0;
              //DMA_TX_DATA_OUT  <= 32'h0;
              error            <= 1;
              error_count      <= error_count + 1;
              reset_once       <= 1;
          end
          else if (SATA_RD_HOLD_IN) begin
            DMA_RX_REN_OUT   <= 0;
            CTRL_READ_EN     <= 0;
            CTRL_WRITE_EN    <= 0;
            test_logic_state <= read_DMA;
            read_count       <= read_count;
          end
          else if (read_count < {test_sector_count_reg << 1, 8'h00}) begin
            test_logic_state  <= read_DMA1;
            if (read_count == ({test_sector_count_reg << 1, 8'h00} - 4)) begin
              DMA_RX_REN_OUT    <= 0;
            end
            else begin
              DMA_RX_REN_OUT    <= 1;
            end
            CTRL_READ_EN      <= 0;
            CTRL_WRITE_EN     <= 0;
            read_count        <= read_count + 4;
            rx_ram_wr_port_en <= 1;
          end
          else begin
            test_logic_state <= read_busy_bit_after_write_DMA;
            DMA_RX_REN_OUT   <= 0;
            CTRL_READ_EN     <= 0;
            CTRL_WRITE_EN    <= 0;
            //read_count       <= 0;
            timeout_count_en    <= 1;
            timeout_count_reset <= 1;
          end
        end
        write_DMA: begin
          if (DMA_TERMINATED) begin
            test_logic_state <= read_busy_bit_after_write_DMA;
            CTRL_READ_EN     <= 0;
            CTRL_WRITE_EN    <= 0;
            DMA_TX_WEN_OUT   <= 0;
            read_count       <= 32'h0;
            timeout_count_en    <= 1;
            timeout_count_reset <= 1;
            error            <= 1;
            error_count      <= error_count + 1;
            reset_once       <= 0;
            //DMA_TX_DATA_OUT  <= 32'h0;
          end
          else if (R_ERR || ILLEGAL_STATE) begin
              test_logic_state <= last_state;
              CTRL_READ_EN     <= 0;
              CTRL_WRITE_EN    <= 0;
              DMA_TX_WEN_OUT   <= 0;
              read_count       <= 32'h0;
              //DMA_TX_DATA_OUT  <= 32'h0;
              error            <= 1;
              error_count      <= error_count + 1;
              reset_once       <= 1;
          end
          else if (SATA_WR_HOLD_IN)begin
            CTRL_WRITE_EN      <= 0;
            DMA_TX_WEN_OUT     <= 0;
            if ((throughput_count == 32'h 00FFFFFF)) begin
              test_logic_state <= last_state;
              error            <= 1;
              error_count      <= error_count + 1;
              reset_once       <= 1;
            end
            else begin
              test_logic_state <= write_DMA;
            end
            read_count         <= read_count;
            //DMA_TX_DATA_OUT    <= DMA_TX_DATA_OUT;
          end
          else if (read_count < ({test_sector_count_reg << 1, 8'h00})) begin
            test_logic_state <= write_DMA;
            CTRL_WRITE_EN    <= 0;
            DMA_TX_WEN_OUT   <= 1;
            read_count       <= read_count + 4;
            //DMA_TX_DATA_OUT  <= read_count;
          end
          else begin
            test_logic_state <= read_busy_bit_after_write_DMA;
            CTRL_READ_EN        <= 0;
            CTRL_WRITE_EN       <= 0;
            DMA_TX_WEN_OUT      <= 0;
            //DMA_TX_DATA_OUT   <= 32'h0;
            tx_ram_rd_en        <= 0;
            timeout_count_en    <= 1;
            timeout_count_reset <= 1;
          end
        end
        
        read_busy_bit_after_write_DMA: begin
          timeout_count_reset  <= 0;
          CTRL_READ_EN      <= 1'b 1;              
          CTRL_WRITE_EN     <= 1'b 0;              
          CTRL_ADDR_REG     <= 5'h 4;
          test_logic_state  <= check_for_BSY_3;
        end
        
        
        check_for_BSY_3: begin
          
          if ((timeout_count == 32'h 004FFFFF)) begin
              test_logic_state <= last_state;
              error            <= 1;
              error_count      <= error_count + 1;
              timeout_count_en <= 0;
              reset_once       <= 1;
          end
          else 
          if(CTRL_DATA_IN[7] == 1'b1)begin
            test_logic_state <= check_for_BSY_3;
            CTRL_READ_EN      <= 1'b 1;              
            CTRL_WRITE_EN     <= 1'b 0;              
            CTRL_ADDR_REG     <= 5'h 4;
          end
          else begin
            read_count       <= 32'h0;
            CTRL_READ_EN     <= 1'b 0;
            CTRL_WRITE_EN    <= 1'b 0;
            test_logic_state <= last_state;
            error_from_SSD   <= CTRL_DATA_IN[0];
            if (CTRL_DATA_IN[0] == 1) begin
              error            <= 1;
              error_count      <= error_count + 1;
              reset_once       <= 1;
            end
            else begin
              error_count      <= error_count;
            end
          end        
        end 
        
        last_state: begin
          CTRL_READ_EN        <= 0;
          CTRL_WRITE_EN       <= 0;
          CTRL_ADDR_REG       <= 5'h 0;
          test_logic_state    <= last_state_2;
          throughput_count_en <= 0;
          read_count          <= 32'h 0;
          reset_controller    <= 0;
          tx_ram_rd_en        <= 0;
          rx_ram_wr_port_en   <= 0;
          timeout_count_en    <= 0;
        end
                
        last_state_2: begin
          CTRL_READ_EN        <= 0;
          CTRL_WRITE_EN       <= 0;
          CTRL_ADDR_REG       <= 5'h 0;
          test_logic_state    <= last_state_3;
          throughput_count_en <= 0;
          read_count          <= 32'h 0;
          reset_controller    <= 0;
          tx_ram_rd_en        <= 0;
          rx_ram_wr_port_en   <= 0;
        end
        
        last_state_3: begin
          CTRL_READ_EN        <= 0;
          CTRL_WRITE_EN       <= 0;
          CTRL_ADDR_REG       <= 5'h 0;
          test_logic_state    <= last_state_4;
          throughput_count_en <= 0;
          read_count          <= 32'h 0;
          reset_controller    <= 0;
          tx_ram_rd_en        <= 0;
          rx_ram_wr_port_en   <= 0;
        end
        
        last_state_4: begin
          CTRL_READ_EN        <= 0;
          CTRL_WRITE_EN       <= 0;
          CTRL_ADDR_REG       <= 5'h 0;
          test_logic_state    <= last_state_5;
          throughput_count_en <= 0;
          read_count          <= 32'h 0;
          reset_controller    <= 0;
          tx_ram_rd_en        <= 0;
          rx_ram_wr_port_en   <= 0;
        end
        
         last_state_5: begin
          CTRL_READ_EN        <= 0;
          CTRL_WRITE_EN       <= 0;
          CTRL_ADDR_REG       <= 5'h 0;
          test_logic_state    <= last_state_6;
          throughput_count_en <= 0;
          read_count          <= 32'h 0;
          reset_controller    <= 0;
          tx_ram_rd_en        <= 0;
          rx_ram_wr_port_en   <= 0;
        end
        
         last_state_6: begin
          CTRL_READ_EN        <= 0;
          CTRL_WRITE_EN       <= 0;
          CTRL_ADDR_REG       <= 5'h 0;
          test_logic_state    <= wait_for_cmd;
          throughput_count_en <= 0;
          read_count          <= 32'h 0;
          reset_controller    <= 0;
          tx_ram_rd_en        <= 0;
          rx_ram_wr_port_en   <= 0;
        end
        
        default: begin
          CTRL_READ_EN        <= 0;
          CTRL_WRITE_EN       <= 0;
          CTRL_ADDR_REG       <= 5'h 0;
          test_logic_state    <= wait_for_cmd;
          throughput_count_en <= 0;
          read_count          <= 32'h 0;
          reset_controller    <= 0;
          tx_ram_rd_en        <= 0;
          rx_ram_wr_port_en   <= 0;
        end
      endcase
    end    
  end //always
  
  assign DMA_TX_DATA_OUT = ping_pong_tx_rd_en ? DMA_TX_DATA_OUT1 :DMA_TX_DATA_OUT2 ;
  
  TEST_TX_DP_RAM TX_RAM1 (
    .clka   (USR_CLOCK),                     // input clka
    .ena    (ping_pong_tx_wr_en),            // input ena
    .wea    (WR_EN_IN),                      // input [0 : 0] wea
    .addra  (ADDRESS_IN[(ADDR_WIDTH-1):2]),  // input [14 : 0] addra
    .dina   (DATA_IN),                       // input [31 : 0] dina
    .clkb   (CLK),                           // input clkb
    .enb    (ping_pong_tx_rd_en),            // input enb
    .addrb  (read_count[(ADDR_WIDTH-1):2]),  // input [14 : 0] addrb
    .doutb  (DMA_TX_DATA_OUT1)               // output [31 : 0] doutb
  );
  TEST_TX_DP_RAM TX_RAM2 (
    .clka   (USR_CLOCK),                     // input clka
    .ena    (!ping_pong_tx_wr_en),           // input ena
    .wea    (WR_EN_IN),                      // input [0 : 0] wea
    .addra  (ADDRESS_IN[(ADDR_WIDTH-1):2]),  // input [14 : 0] addra
    .dina   (DATA_IN),                       // input [31 : 0] dina
    .clkb   (CLK),                           // input clkb
    .enb    (!ping_pong_tx_rd_en),           // input enb
    .addrb  (read_count[(ADDR_WIDTH-1):2]),  // input [14 : 0] addrb
    .doutb  (DMA_TX_DATA_OUT2)               // output [31 : 0] doutb
  );
 
  assign rx_ram_wr_addr = read_count; // - 4;
  
  assign rx_ram1_sel = rx_ram_wr_port_en & ping_pong_rx_wr_en;
   
  TEST_TX_DP_RAM RX_RAM1 (
    .clka   (CLK),                               // input clka
    .ena    (rx_ram1_sel),                       // input ena
    .wea    (DMA_RX_REN_OUT),                    // input [0 : 0] wea
    .addra  (rx_ram_wr_addr[(ADDR_WIDTH-1):2]),  // input [14 : 0] addra
    .dina   (DMA_RX_DATA_IN),                    // input [31 : 0] dina
    .clkb   (USR_CLOCK),                         // input clkb
    .enb    (ping_pong_rx_rd_en),                // input enb
    .addrb  (ADDRESS_IN[(ADDR_WIDTH-1):2]),      // input [14 : 0] addrb
    .doutb  (rx_ram_dout1)                       // output [31 : 0] doutb
  );
  
  TEST_TX_DP_RAM RX_RAM2 (
    .clka   (CLK),                               // input clka
    .ena    (!rx_ram1_sel),                      // input ena
    .wea    (DMA_RX_REN_OUT),                    // input [0 : 0] wea
    .addra  (rx_ram_wr_addr[(ADDR_WIDTH-1):2]),  // input [14 : 0] addra
    .dina   (DMA_RX_DATA_IN),                    // input [31 : 0] dina
    .clkb   (USR_CLOCK),                         // input clkb
    .enb    (!ping_pong_rx_rd_en),               // input enb
    .addrb  (ADDRESS_IN[(ADDR_WIDTH-1):2]),      // input [14 : 0] addrb
    .doutb  (rx_ram_dout2)                       // output [31 : 0] doutb
  );
  
  assign DATA_OUT = (ping_pong_rx_rd_en == 1'b1)? rx_ram_dout1 : rx_ram_dout2;
  

  // Instantiate the module
  SATA_CONTROLLER SATA_CONTROLLER1(
    .TILE0_REFCLK_PAD_P_IN  (TILE0_REFCLK_PAD_P_IN), 
    .TILE0_REFCLK_PAD_N_IN  (TILE0_REFCLK_PAD_N_IN), 
    .GTPRESET_IN            (SATA_RESET_OUT),  
    .TILE0_PLLLKDET_OUT     (TILE0_PLLLKDET_OUT), 
    .TXP0_OUT               (TXP0_OUT), 
    .TXN0_OUT               (TXN0_OUT), 
    .RXP0_IN                (RXP0_IN), 
    .RXN0_IN                (RXN0_IN), 
    .DCMLOCKED_OUT          (DCMLOCKED_OUT), 
    .LINKUP                 (LINKUP), 
    .GEN                    (GEN), 
 // .PHY_CLK_OUT            (PHY_CLK_OUT), 
    .CLK_OUT                (CLK), 
    .HOST_READ_EN           (CTRL_READ_EN), 
    .HOST_WRITE_EN          (CTRL_WRITE_EN), 
    .HOST_ADDR_REG          (CTRL_ADDR_REG), 
    .HOST_DATA_IN           (CTRL_DATA_OUT),
    .HOST_DATA_OUT          (CTRL_DATA_IN),
    .RESET_OUT              (RESET),
    .WRITE_HOLD_U           (SATA_WR_HOLD_IN),
    .READ_HOLD_U            (SATA_RD_HOLD_IN),
    .PIO_CLK_IN             (CLK),
    .DMA_CLK_IN             (CLK),
    .DMA_RQST               (DMA_RQST_OUT),
    .DMA_RX_DATA_OUT        (DMA_RX_DATA_IN),
    .DMA_RX_REN             (DMA_RX_REN_OUT),
    .DMA_TX_DATA_IN         (DMA_TX_DATA_OUT),
    .DMA_TX_WEN             (DMA_TX_WEN_OUT),
    .CE                     (1'b1),
    .IPF                    (INTERRUPT_IN),
    .DMA_TERMINATED         (DMA_TERMINATED),
    .R_ERR                  (R_ERR),
    .ILLEGAL_STATE          (ILLEGAL_STATE),
    .RX_FIFO_RESET          (RX_FIFO_RESET_OUT),
    .TX_FIFO_RESET          (RX_FIFO_RESET_OUT),
    .DMA_DATA_RCV_ERROR     (DMA_DATA_RCV_ERROR),
    .OOB_reset_IN           (OOB_reset_IN),
    .RX_FSM_reset_IN        (RX_FSM_reset_IN),
		.TX_FSM_reset_IN        (TX_FSM_reset_IN)    
    
    
    );


endmodule
