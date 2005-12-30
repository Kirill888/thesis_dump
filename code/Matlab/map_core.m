function m_core = map_core(mm)

  nm = size(mm,2);
  m_core = mm;
  
  mlast = size(mm(1).map,2);
  
  for i = 1:nm
    ind_core = find(mm(i).map(:,mlast) == 3);
    
    m_core(i).map = mm(i).map(ind_core,:);
  end
  