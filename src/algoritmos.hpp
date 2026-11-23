// Split, SplitLazy (Algoritmo 1 de Hokama, Lintzmayer e San Felice, 2024) e ótimo exato do FSTSP.
// Operação o_{i,j,k}: lançamento na posição i, cliente do drone em j e encontro em k (i < j < k).
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "instancia.hpp"

const double INFINITO = std::numeric_limits<double>::infinity();

struct Operacao {
    int lancamento;
    int drone;   // -1 quando o caminhão anda sozinho de i a i+1
    int encontro;
};

struct Solucao {
    double makespan = INFINITO;
    long long triplas = 0;
    std::vector<Operacao> operacoes;
};

inline std::vector<Operacao> refazer_operacoes(const std::vector<int>& anterior,
                                               const std::vector<int>& drone_do_arco) {
    std::vector<Operacao> operacoes;
    for (int k = (int)anterior.size() - 1; k > 0; k = anterior[k])
        operacoes.push_back({anterior[k], drone_do_arco[k], k});
    std::reverse(operacoes.begin(), operacoes.end());
    return operacoes;
}

inline Solucao split(const Instancia& inst, const std::vector<int>& h) {
    const int n = inst.num_clientes;
    const double alpha = inst.alpha;
    auto t = [&](int a, int b) { return inst.tempo[h[a]][h[b]]; };

    std::vector<double> acumulado(n + 2, 0.0);
    for (int a = 1; a <= n + 1; ++a) acumulado[a] = acumulado[a - 1] + t(a - 1, a);

    std::vector<double> menor_tempo(n + 2, INFINITO);
    std::vector<int> anterior(n + 2, -1), drone_do_arco(n + 2, -1);
    menor_tempo[0] = 0.0;
    Solucao sol;

    for (int k = 1; k <= n + 1; ++k) {
        menor_tempo[k] = menor_tempo[k - 1] + t(k - 1, k);
        anterior[k] = k - 1;
        drone_do_arco[k] = -1;

        for (int i = 0; i <= k - 2; ++i) {
            double custo_arco = INFINITO;
            int melhor_j = -1;
            for (int j = i + 1; j <= k - 1; ++j) {
                double caminhao = acumulado[k] - acumulado[i]
                                - t(j - 1, j) - t(j, j + 1) + t(j - 1, j + 1);
                double voo = (t(i, j) + t(j, k)) / alpha;
                double custo = std::max(caminhao, voo);
                ++sol.triplas;
                if (custo < custo_arco) { custo_arco = custo; melhor_j = j; }
            }
            if (menor_tempo[i] + custo_arco < menor_tempo[k]) {
                menor_tempo[k] = menor_tempo[i] + custo_arco;
                anterior[k] = i;
                drone_do_arco[k] = melhor_j;
            }
        }
    }
    sol.makespan = menor_tempo[n + 1];
    sol.operacoes = refazer_operacoes(anterior, drone_do_arco);
    return sol;
}

struct ArcoLazy {
    int origem;
    int drone;
    double custo;
    int proximo;
};

inline Solucao split_lazy(const Instancia& inst, const std::vector<int>& h) {
    const int n = inst.num_clientes;
    const double alpha = inst.alpha;
    auto t = [&](int a, int b) { return inst.tempo[h[a]][h[b]]; };

    std::vector<int> primeiro_arco(n + 2, -1);
    std::vector<ArcoLazy> arcos;
    arcos.reserve(4 * (n + 2));

    auto adicionar_ou_atualizar = [&](int i, int k, double custo, int j) {
        for (int a = primeiro_arco[k]; a != -1; a = arcos[a].proximo)
            if (arcos[a].origem == i) {
                if (arcos[a].custo > custo) { arcos[a].custo = custo; arcos[a].drone = j; }
                return;
            }
        arcos.push_back({i, j, custo, primeiro_arco[k]});
        primeiro_arco[k] = (int)arcos.size() - 1;
    };

    for (int i = 0; i <= n; ++i) {
        arcos.push_back({i, -1, t(i, i + 1), primeiro_arco[i + 1]});
        primeiro_arco[i + 1] = (int)arcos.size() - 1;
    }

    Solucao sol;
    for (int j = 1; j <= n; ++j) {
        double custo_base = t(j - 1, j + 1);
        double custo_antes = -t(j - 1, j);
        int k_max = n + 1;
        for (int i = j - 1; i >= 0; --i) {
            int k = j + 1;
            double custo_drone = (t(i, j) + t(j, k)) / alpha;
            custo_antes += t(i, i + 1);
            double custo_depois = 0.0;
            double custo_caminhao = custo_antes + custo_base + custo_depois;
            ++sol.triplas;
            adicionar_ou_atualizar(i, k, std::max(custo_drone, custo_caminhao), j);
            if (custo_drone <= custo_caminhao) break;

            for (k = j + 2; k <= k_max; ++k) {
                custo_drone = (t(i, j) + t(j, k)) / alpha;
                custo_depois += t(k - 1, k);
                custo_caminhao = custo_antes + custo_base + custo_depois;
                ++sol.triplas;
                adicionar_ou_atualizar(i, k, std::max(custo_drone, custo_caminhao), j);
                if (custo_drone <= custo_caminhao) {
                    k_max = k - 1;
                    break;
                }
            }
        }
    }

    std::vector<double> menor_tempo(n + 2, INFINITO);
    std::vector<int> anterior(n + 2, -1), drone_do_arco(n + 2, -1);
    menor_tempo[0] = 0.0;
    for (int k = 1; k <= n + 1; ++k)
        for (int a = primeiro_arco[k]; a != -1; a = arcos[a].proximo) {
            double candidato = menor_tempo[arcos[a].origem] + arcos[a].custo;
            if (candidato < menor_tempo[k]) {
                menor_tempo[k] = candidato;
                anterior[k] = arcos[a].origem;
                drone_do_arco[k] = arcos[a].drone;
            }
        }
    sol.makespan = menor_tempo[n + 1];
    sol.operacoes = refazer_operacoes(anterior, drone_do_arco);
    return sol;
}

// Recalcula o makespan trecho a trecho, sem somas prefixas. Devolve NaN se a solução for inválida.
inline double avaliar_solucao(const Instancia& inst, const std::vector<int>& h,
                              const std::vector<Operacao>& operacoes) {
    const int n = inst.num_clientes;
    auto t = [&](int a, int b) { return inst.tempo[h[a]][h[b]]; };
    double total = 0.0;
    int posicao = 0;
    for (const Operacao& op : operacoes) {
        if (op.lancamento != posicao || op.encontro <= op.lancamento) return NAN;
        if (op.drone == -1) {
            if (op.encontro != op.lancamento + 1) return NAN;
            total += t(op.lancamento, op.encontro);
        } else {
            if (op.drone <= op.lancamento || op.drone >= op.encontro) return NAN;
            double caminhao = 0.0;
            int ultimo = op.lancamento;
            for (int a = op.lancamento + 1; a <= op.encontro; ++a) {
                if (a == op.drone) continue;
                caminhao += t(ultimo, a);
                ultimo = a;
            }
            double voo = (t(op.lancamento, op.drone) + t(op.drone, op.encontro)) / inst.alpha;
            total += std::max(caminhao, voo);
        }
        posicao = op.encontro;
    }
    return posicao == n + 1 ? total : NAN;
}

inline bool mesmo_valor(double a, double b) {
    return std::fabs(a - b) <= 1e-9 * std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
}

// Ótimo exato: menor valor do h-FSTSP entre os n! ciclos com o depósito fixo.
struct ResultadoExato {
    double otimo = INFINITO;
    long long ciclos_enumerados = 0;
    long long divergencias_split = 0;
    std::vector<int> melhor_ciclo;
};

inline ResultadoExato otimo_por_enumeracao(const Instancia& inst, bool conferir_com_split) {
    const int n = inst.num_clientes;
    std::vector<int> h(n + 2, 0), clientes(n);
    for (int c = 0; c < n; ++c) clientes[c] = c + 1;

    ResultadoExato res;
    do {
        std::copy(clientes.begin(), clientes.end(), h.begin() + 1);
        double valor = split_lazy(inst, h).makespan;
        if (conferir_com_split && !mesmo_valor(valor, split(inst, h).makespan))
            ++res.divergencias_split;
        if (valor < res.otimo) { res.otimo = valor; res.melhor_ciclo = h; }
        ++res.ciclos_enumerados;
    } while (std::next_permutation(clientes.begin(), clientes.end()));
    return res;
}
