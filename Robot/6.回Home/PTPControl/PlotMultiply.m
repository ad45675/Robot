clc ; clear ; close all ;


%% 讀取實驗資料

MeasuredData = load( 'MeasuredData.txt' ) ;

mod = 2;
switch mod
    case 1
        
        
        Pos = MeasuredData( : , 1 : 6 ) ;
        Vel = MeasuredData( : , 7 : 12 ) ;
        PosCmd = MeasuredData( : , 13 : 18 ) ;
        VelCmd = MeasuredData( : , 19 : 24 ) ;
        TorCtrl = MeasuredData( : , 25 : 30 ) ;
        
        PosError = PosCmd - Pos ;
        VelError = VelCmd - Vel ;
        
        
        %% 時間
        
        n = 6 ;
        st = 0.001 ;
        tf = size( MeasuredData , 1 ) * st ;
        Time = st : st : tf ;
        
        
        
        %%
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
            plot( Time , TorCtrl( : , i ) , '-' , 'LineWidth' , 1 ) ;
            title( [ 'TorCtrl ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
            xlabel( 'Time (sec)') ; ylabel( 'Tor (Nm)', 'FontSize' , 10 ) ;
            legend( 'TorCtrl' ) ;
            grid on ;
            
        end
    case 2
        Data = load('MeasuredData.txt');
        MeasuredData = struct('Pos', Data( : , 1 : 6 ),...
            'Vel', Data( : , 7 : 12 ),...
            'PosCmd', Data( : , 13 : 18 ),...
            'VelCmd',Data( : , 19 : 24 ),...
            'TorCtrl',Data( : , 25 : 30 ),...
            'PosError', Data( : , 13 : 18 ) - Data( : , 1 : 6 ),...
            'VelError' , Data( : , 19 : 24 ) - Data( : , 7 : 12 ),...
            'Time', (0.001: 0.001: length(Data(:,1)) * 0.001)');
        
        
        save_pic = false;
        linecolors={'r' 'b' 'g' [0 .5 0] };
        p = ['Pos', 'PosCmd',  'Vel', 'TorCtrl'];
        
        
        for i = 1 : 6
            h3i = plotNy( MeasuredData.Time( : ), MeasuredData.Pos( : , i ),1,...
                MeasuredData.Time( : ), MeasuredData.PosCmd( : , i ),2,...
                MeasuredData.Time( : ),MeasuredData.Vel( : , i ),3,...
                MeasuredData.Time( : ),MeasuredData.TorCtrl( : , i ),4,...
                'Linewidth',1,...
                'YAxisLabels',{'Pos[rad]' 'PosCmd[rad]' 'Vel[rad/s]' 'TorCtrl[N-m]' },...
                'XAxisLabel','Time [s]',...
                'TitleStr',sprintf('Joint%i', i),...
                'LineColor',linecolors,...
                'FontSize',12,...
                'Fontname','TimesNewRoman',...
                'Grid','on',...
                'LegendString',{'Pos[rad]' 'PosCmd[rad]' 'Vel[rad/s]' 'TorCtrl[N-m]'});
            for k=1:length(h3i.ax)
                set(h3i.ax(k),'ycolor',linecolors{k});
            end
            set(h3i.parent, 'Position', [0.1, 0.1, 0.8, 0.6])
            if save_pic
                saveas(h3i.parent,['Fig\',sprintf('Data%i_Joint%i', pick, i), '.jpg']);
            end
            clear h3i
        end
end