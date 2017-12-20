    
 z=plan_path;
    for n=1:size(z,1)
        for l=1:size(obs_ref,1)  
               if  ~(((z(n,1)-obs_ref(l,1))^2+(z(n,2)-obs_ref(l,2))^2)>=0.7)
                   disp('fail to obstacle avoid')
                   break          
               end
        end
    end