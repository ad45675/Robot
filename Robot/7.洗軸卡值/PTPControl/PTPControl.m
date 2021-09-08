clc ; clear ; close all ;

%% 讀取實驗資料

MeasuredData = load( 'MeasuredData.txt' ) ;

Pos = MeasuredData( : , 1 : 6 ) ;
Vel = MeasuredData( : , 7 : 12 ) ; 
PosCmd = MeasuredData( : , 13 : 18 ) ; 
VelCmd = MeasuredData( : , 19 : 24 ) ;
TorCtrl = MeasuredData( : , 25 : 30 ) ;
TorM = MeasuredData( : , 31 : 36 ) ; 
TorN = MeasuredData( : , 37 : 42 ) ;
TorF = MeasuredData( : , 43 : 48 ) ;

PosError = PosCmd - Pos ;
VelError = VelCmd - Vel ;
TorFF = TorM + TorN + TorF ;


%% 時間

n = 6 ;
st = 0.001 ;
tf = size( MeasuredData , 1 ) * st ;
Time = st : st : tf ;


%% 繪製結果圖

figure( 1 )

for i = 1 : n
    
    subplot( 3 , 2 , i ) ;
    plot( Time , PosCmd( : , i ) , '-' , Time , Pos( : , i ) , '--' , 'LineWidth' , 1 ) ;
    title( [ 'Pos ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Pos (rad)', 'FontSize' , 10 ) ;
    legend( 'PosCmd' , 'Pos' ) ;
    grid on ;
    
end

figure( 2 )

for i = 1 : n
    
    subplot( 3 , 2 , i ) ;
    plot( Time , PosError( : , i ) , '-' , 'LineWidth' , 1 ) ;
    title( [ 'PosError ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel('Time (sec)') ; ylabel( 'Error (rad)', 'FontSize' , 10 ) ;
    grid on ;
    
end

figure( 3 )

for i = 1 : n
    
    subplot( 3 , 2 , i ) ;
    plot( Time , VelCmd( : , i ) , '-' , Time , Vel( : , i ) , '--' , 'LineWidth' , 1 ) ;
    title( [ 'Vel ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Vel (rad/s)', 'FontSize' , 10 ) ;
    legend( 'VelCmd' , 'Vel' ) ;
    grid on ;
    
end

figure( 4 )

for i = 1 : n
    
    subplot( 3 , 2 , i ) ;
    plot( Time , VelError( : , i ) , '-' , 'LineWidth' , 1 ) ;
    title( [ 'VelError ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Error (rad/s)', 'FontSize' , 10 ) ;
    grid on ;
    
end

figure( 5 )

for i = 1 : n
    
    subplot( 3 , 2 , i ) ;
    plot( Time , TorCtrl( : , i ) , '-' , Time , TorFF( : , i ) , '--' , 'LineWidth' , 1 ) ;
    title( [ 'TorCtrl ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Tor (Nm)', 'FontSize' , 10 ) ;
    legend( 'TorCtrl' , 'TorFF' ) ;
    grid on ;
    
end



