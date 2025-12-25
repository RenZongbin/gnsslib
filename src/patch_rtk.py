import re

file_path = 'rtk_common.cpp'

with open(file_path, 'r') as f:
    content = f.read()

start_marker = "static int resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa)"

new_body = """{
        prcopt_t *opt = &rtk->opt;
        int i, j, k, nb, info, n, m=2;
        int idx_amb = NP(opt) + NI(opt) + NT(opt) + NL(opt);
        int nb_tot = rtk->nx - idx_amb;
        
        fprintf(stderr, "DEBUG PAR early: nx=%d, idx_amb=%d, nb_tot=%d\\n", rtk->nx, idx_amb, nb_tot);

        if (rtk->sol.stat != SOLQ_FLOAT || nb_tot <= 0) return 0;
        
        // Debug first valid ambiguities (unchecked var)
        for (i=0; i<nb_tot && i<10; i++) {
             int p_idx = (idx_amb+i) + (idx_amb+i)*rtk->nx;
             double var = rtk->P[p_idx];
             double x_val = rtk->x[idx_amb+i];
             fprintf(stderr, "DEBUG PAR: Amb %d (idx=%d): x=%.3f P=%.3f\\n", i, idx_amb+i, x_val, var);
        }

        // PAR: Select usage ambiguities
        std::vector<int> ix;
        for (i=0; i<nb_tot; i++) {
            // Check if ambiguity is valid (variance > 0 and not huge)
            if (rtk->P[(idx_amb+i) + (idx_amb+i)*rtk->nx] > 0.0 &&
                rtk->P[(idx_amb+i) + (idx_amb+i)*rtk->nx] < 1000.0) {
                ix.push_back(i);
            }
        }
        
        fprintf(stderr, "DEBUG PAR: nb_tot=%d, valid_amb=%lu\\n", nb_tot, ix.size());
        
        // Iterative PAR
        double *y, *Qb, *Qab, *b, s[2];
        int ny = rtk->nx; // Full state size
        
        while (ix.size() >= 4) { // Minimum 4 ambiguities to fix
            n = ix.size();
            y = mat(n, 1);
            Qb = mat(n, n);
            Qab = mat(idx_amb, n);
            b = mat(n, m);
            
            for (i=0; i<n; i++) {
                y[i] = rtk->x[idx_amb + ix[i]];
                for (j=0; j<n; j++) {
                    Qb[i+j*n] = rtk->P[(idx_amb + ix[i]) + (idx_amb + ix[j])*ny];
                }
                for (j=0; j<idx_amb; j++) {
                    Qab[j+i*idx_amb] = rtk->P[j + (idx_amb + ix[i])*ny];
                }
            }
            
            // LAMBDA
            if (!(info = lambda(n, 2, y, Qb, b, s))) {
                rtk->sol.ratio = s[0] > 0 ? s[1]/s[0] : 0;
                fprintf(stderr, "DEBUG PAR: n=%d, ratio=%.2f, thres=%.2f\\n", n, rtk->sol.ratio, opt->thresar[0]);
                if (rtk->sol.ratio > opt->thresar[0]) {
                    // Fixed!
                    rtk->sol.stat = SOLQ_FIX;
                    matcpy(xa, rtk->x, ny, 1);
                    
                    // Update fixed ambiguities
                    for(i=0;i<n;i++) xa[idx_amb + ix[i]] = b[i];
                    
                    // Update other states
                    Eigen::Map<Eigen::MatrixXd> Qb_mat(Qb, n, n);
                    Eigen::Map<Eigen::MatrixXd> Qab_mat(Qab, idx_amb, n);
                    Eigen::Map<Eigen::VectorXd> y_vec(y, n);
                    Eigen::Map<Eigen::VectorXd> b_vec(b, n);
                    
                    Eigen::VectorXd diff = b_vec - y_vec;
                    Eigen::VectorXd dx = Qab_mat * Qb_mat.ldlt().solve(diff);
                    
                    for(i=0;i<idx_amb;i++) xa[i] += dx(i);
                    
                    delete[] y; delete[] Qb; delete[] Qab; delete[] b;
                    return 1;
                }
            } else {
                rtk->sol.ratio = 0.0;
                fprintf(stderr, "DEBUG PAR: lambda failed info=%d\\n", info);
            }
            
            delete[] y; delete[] Qb; delete[] Qab; delete[] b;
            
            // Remove worst ambiguity
            int worst_idx = -1;
            double max_var = -1.0;
            for (i=0; i<n; i++) {
                double var = rtk->P[(idx_amb + ix[i]) + (idx_amb + ix[i])*ny];
                if (var > max_var) {
                    max_var = var;
                    worst_idx = i;
                }
            }
            if (worst_idx >= 0) {
                ix.erase(ix.begin() + worst_idx);
            } else {
                break;
            }
        }
        
        rtk->sol.stat = SOLQ_FLOAT;
        return 0;
    }"""
    
lines = content.splitlines(keepends=True)
start_idx = -1
end_idx = -1

for i, line in enumerate(lines):
    if "static int resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa)" in line:
        start_idx = i
        break

if start_idx != -1:
    brace_count = 0
    found_start = False
    for i in range(start_idx, len(lines)):
        line = lines[i]
        brace_count += line.count('{')
        if '{' in line:
            found_start = True
        brace_count -= line.count('}')
        
        if found_start and brace_count == 0:
            end_idx = i
            break

if start_idx != -1 and end_idx != -1:
    print(f"Replacing lines {start_idx+1} to {end_idx+1}")
    new_lines = [lines[start_idx]] + [new_body + "\n"] + lines[end_idx+1:]
    final_content = "".join(lines[:start_idx+1]) + new_body + "\n" + "".join(lines[end_idx+1:])
    
    with open(file_path, 'w') as f:
        f.write(final_content)
    print("Successfully patched resamb_LAMBDA with debugs V4")
else:
    print("Could not find function bounds")
