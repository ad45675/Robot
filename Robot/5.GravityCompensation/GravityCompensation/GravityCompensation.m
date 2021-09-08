clc ; clear ; close all ;

%% Ū��������

MeasuredData = load( 'MeasuredData.txt' ) ;

Pos = MeasuredData( : , 1 : 6 ) ;
Vel = MeasuredData( : , 7 : 12 ) ; 
TorG = MeasuredData( : , 13 : 18 ) ;


%% �ɶ�

n = 6 ;
st = 0.001 ;
tf = size( MeasuredData , 1 ) * st ;
Time = st : st : tf ;


%% ø�s���G��

figure( 1 )

for i = 1 : n
    
    subplot( 3 , 2 , i ) ;
    plot( Time , Pos( : , i ) , '-' , 'LineWidth' , 1 ) ;
    title( [ 'Pos ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Pos (rad)', 'FontSize' , 10 ) ;
    grid on ;
    
end

figure( 2 )

for i = 1 : n
    
    subplot( 3 , 2 , i ) ;
    plot( Time , Vel( : , i ) , '-' , 'LineWidth' , 1 ) ;
    title( [ 'Vel ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Vel (rad/s)', 'FontSize' , 10 ) ;
    grid on ;
    
end

figure( 3 )

for i = 1 : n
    
    subplot( 3 , 2 , i ) ;
    plot( Time , TorG( : , i ) , '--' , 'LineWidth' , 1 ) ;
    title( [ 'TorCtrl ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Tor (Nm)', 'FontSize' , 10 ) ;
    grid on ;
    
end



