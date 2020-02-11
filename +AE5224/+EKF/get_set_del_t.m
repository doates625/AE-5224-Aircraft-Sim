function del_t = get_set_del_t(del_t_new)
%GET_SET_DT Set or get EKF time delta [s]
%   del_t = GET_SET_DT(del_t_new) Set del_t to del_t_new
%   del_t = GET_SET_DT() Get dt

persistent del_t_
if nargin
    del_t_ = del_t_new;
end
del_t = del_t_;

end