// Leitura das instâncias de Bouman, Agatz e Schmidt (2018) e ciclo do Vizinho Mais Próximo.
#pragma once

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Instancia {
    std::string conjunto, nome;
    int num_clientes = 0;
    double alpha = 1.0;
    std::vector<std::vector<double>> tempo;
};

inline std::vector<std::string> ler_palavras(const std::string& caminho) {
    std::ifstream arquivo(caminho);
    if (!arquivo) throw std::runtime_error("nao foi possivel abrir " + caminho);
    std::stringstream buffer;
    buffer << arquivo.rdbuf();
    std::string texto = buffer.str(), limpo;

    for (size_t p = 0; p < texto.size(); ++p) {
        if (texto[p] == '/' && p + 1 < texto.size() && texto[p + 1] == '*') {
            size_t fim = texto.find("*/", p + 2);
            if (fim == std::string::npos) break;
            p = fim + 1;
            limpo += ' ';
        } else {
            limpo += texto[p];
        }
    }
    std::vector<std::string> palavras;
    std::istringstream fluxo(limpo);
    for (std::string palavra; fluxo >> palavra;) palavras.push_back(palavra);
    return palavras;
}

inline Instancia ler_instancia(const std::string& caminho, const std::string& conjunto,
                               const std::string& nome) {
    std::vector<std::string> p = ler_palavras(caminho);
    Instancia inst;
    inst.conjunto = conjunto;
    inst.nome = nome;
    double fator_caminhao = std::stod(p.at(0));
    double fator_drone = std::stod(p.at(1));
    inst.alpha = fator_caminhao / fator_drone;
    int num_nos = std::stoi(p.at(2));
    inst.num_clientes = num_nos - 1;

    std::vector<double> x(num_nos), y(num_nos);
    for (int v = 0; v < num_nos; ++v) {
        x[v] = std::stod(p.at(3 + 3 * v));
        y[v] = std::stod(p.at(4 + 3 * v));
    }
    inst.tempo.assign(num_nos, std::vector<double>(num_nos, 0.0));
    for (int a = 0; a < num_nos; ++a)
        for (int b = 0; b < num_nos; ++b)
            inst.tempo[a][b] = fator_caminhao * std::hypot(x[a] - x[b], y[a] - y[b]);
    return inst;
}

// Ciclo h = (v_0, v_1, ..., v_n, v_{n+1}), com v_0 = v_{n+1} = 0 (depósito).
inline std::vector<int> ler_ciclo(const std::string& caminho, int num_clientes) {
    std::vector<std::string> p = ler_palavras(caminho);
    int num_operacoes = std::stoi(p.at(0));
    std::vector<int> ciclo = {0};
    size_t pos = 1;
    for (int op = 0; op < num_operacoes; ++op) {
        int fim = std::stoi(p.at(pos + 1));
        int internos = std::stoi(p.at(pos + 3));
        for (int q = 0; q < internos; ++q) ciclo.push_back(std::stoi(p.at(pos + 4 + q)));
        ciclo.push_back(fim);
        pos += 4 + internos;
    }

    std::vector<int> visto(num_clientes + 1, 0);
    bool ok = (int)ciclo.size() == num_clientes + 2 && ciclo.front() == 0 && ciclo.back() == 0;
    for (int q = 1; ok && q <= num_clientes; ++q) {
        int v = ciclo[q];
        ok = v >= 1 && v <= num_clientes && !visto[v];
        if (ok) visto[v] = 1;
    }
    if (!ok) throw std::runtime_error("ciclo invalido em " + caminho);
    return ciclo;
}

inline std::vector<int> ciclo_vizinho_mais_proximo(const Instancia& inst) {
    int n = inst.num_clientes, atual = 0;
    std::vector<int> ciclo = {0};
    std::vector<char> visitado(n + 1, 0);
    for (int passo = 0; passo < n; ++passo) {
        int proximo = -1;
        double menor = std::numeric_limits<double>::infinity();
        for (int v = 1; v <= n; ++v)
            if (!visitado[v] && inst.tempo[atual][v] < menor) {
                menor = inst.tempo[atual][v];
                proximo = v;
            }
        visitado[proximo] = 1;
        ciclo.push_back(proximo);
        atual = proximo;
    }
    ciclo.push_back(0);
    return ciclo;
}

inline double comprimento_ciclo(const Instancia& inst, const std::vector<int>& ciclo) {
    double total = 0.0;
    for (size_t a = 0; a + 1 < ciclo.size(); ++a) total += inst.tempo[ciclo[a]][ciclo[a + 1]];
    return total;
}

inline double fracao_1_localidade(const Instancia& inst, const std::vector<int>& ciclo) {
    int n = inst.num_clientes, contagem = 0;
    for (int j = 1; j <= n; ++j) {
        double pulo = inst.tempo[ciclo[j - 1]][ciclo[j + 1]];
        double voo = (inst.tempo[ciclo[j - 1]][ciclo[j]] + inst.tempo[ciclo[j]][ciclo[j + 1]]) / inst.alpha;
        if (pulo >= voo) ++contagem;
    }
    return n > 0 ? double(contagem) / n : 0.0;
}
