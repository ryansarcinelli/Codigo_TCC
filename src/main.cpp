// Uso:
//   fstsp experimento --pasta DIR [--pasta DIR ...] --saida resultados.csv [--tempo-min-ms 20] [--semente 2026]
//   fstsp exato       --pasta DIR [--pasta DIR ...] --saida exato.csv [--max-clientes 9] [--conferir-split]

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "algoritmos.hpp"
#include "instancia.hpp"

namespace fs = std::filesystem;
using Relogio = std::chrono::steady_clock;

struct ArquivoInstancia {
    std::string caminho, conjunto, nome, caminho_ciclo_otimo;
};

// As versões "alpha_X" usam o arquivo -tsp.txt da instância original.
std::string procurar_ciclo_otimo(const fs::path& pasta, const std::string& nome) {
    fs::path direto = pasta / "solutions" / (nome + "-tsp.txt");
    if (fs::exists(direto)) return direto.string();
    size_t p = nome.find("alpha_");
    if (p != std::string::npos) {
        std::string sem_alpha = nome.substr(0, p) + nome.substr(p + 8);
        fs::path alternativo = pasta / "solutions" / (sem_alpha + "-tsp.txt");
        if (fs::exists(alternativo)) return alternativo.string();
    }
    return "";
}

std::vector<ArquivoInstancia> listar_instancias(const std::vector<std::string>& pastas) {
    std::vector<ArquivoInstancia> lista;
    for (const std::string& texto_pasta : pastas) {
        fs::path pasta(texto_pasta);
        std::string conjunto = pasta.filename().string();
        if (conjunto.empty()) conjunto = pasta.parent_path().filename().string();
        for (const auto& item : fs::directory_iterator(pasta)) {
            if (!item.is_regular_file() || item.path().extension() != ".txt") continue;
            std::string nome = item.path().stem().string();
            lista.push_back({item.path().string(), conjunto, nome, procurar_ciclo_otimo(pasta, nome)});
        }
    }
    std::sort(lista.begin(), lista.end(), [](const auto& a, const auto& b) {
        return a.conjunto != b.conjunto ? a.conjunto < b.conjunto : a.nome < b.nome;
    });
    return lista;
}

// Tempo médio de uma chamada (us): aquecimento e lotes dobrados até durar tempo_min_ms.
template <class Funcao>
double medir_microssegundos(Funcao&& decodificar, double tempo_min_ms, long long& repeticoes) {
    volatile double descarte = decodificar().makespan;
    (void)descarte;
    double ms = 0.0;
    for (repeticoes = 1;; repeticoes *= 2) {
        auto inicio = Relogio::now();
        for (long long r = 0; r < repeticoes; ++r) descarte = decodificar().makespan;
        ms = std::chrono::duration<double, std::milli>(Relogio::now() - inicio).count();
        if (ms >= tempo_min_ms) break;
    }
    return ms * 1000.0 / repeticoes;
}

int modo_experimento(const std::vector<std::string>& pastas, const std::string& saida,
                     double tempo_min_ms, unsigned semente) {
    std::vector<ArquivoInstancia> lista = listar_instancias(pastas);
    std::shuffle(lista.begin(), lista.end(), std::mt19937(semente));

    std::ofstream csv(saida);
    csv << "conjunto,instancia,n,alpha,ciclo,comprimento_ciclo,frac_1local,"
           "makespan_split,makespan_lazy,valido,triplas_split,triplas_lazy,"
           "t_split_us,t_lazy_us,reps_split,reps_lazy\n";
    csv << std::setprecision(12);

    int feitas = 0, divergencias = 0;
    for (const ArquivoInstancia& arq : lista) {
        if (arq.caminho_ciclo_otimo.empty()) {
            std::cerr << "ignorada (sem ciclo otimo): " << arq.nome << "\n";
            continue;
        }
        Instancia inst = ler_instancia(arq.caminho, arq.conjunto, arq.nome);
        std::vector<std::pair<std::string, std::vector<int>>> ciclos = {
            {"otimo", ler_ciclo(arq.caminho_ciclo_otimo, inst.num_clientes)},
            {"vmp", ciclo_vizinho_mais_proximo(inst)}};

        for (const auto& par : ciclos) {
            const std::vector<int>& h = par.second;
            Solucao s1 = split(inst, h);
            Solucao s2 = split_lazy(inst, h);
            double v1 = avaliar_solucao(inst, h, s1.operacoes);
            double v2 = avaliar_solucao(inst, h, s2.operacoes);
            bool valido = mesmo_valor(s1.makespan, s2.makespan) && mesmo_valor(v1, s1.makespan) &&
                          mesmo_valor(v2, s2.makespan);
            if (!valido) ++divergencias;

            long long reps_split = 0, reps_lazy = 0;
            double t_split = medir_microssegundos([&] { return split(inst, h); }, tempo_min_ms, reps_split);
            double t_lazy = medir_microssegundos([&] { return split_lazy(inst, h); }, tempo_min_ms, reps_lazy);

            csv << inst.conjunto << ',' << inst.nome << ',' << inst.num_clientes << ','
                << inst.alpha << ',' << par.first << ',' << comprimento_ciclo(inst, h) << ','
                << fracao_1_localidade(inst, h) << ',' << s1.makespan << ',' << s2.makespan << ','
                << (valido ? 1 : 0) << ',' << s1.triplas << ',' << s2.triplas << ','
                << t_split << ',' << t_lazy << ',' << reps_split << ',' << reps_lazy << '\n';
        }
        if (++feitas % 100 == 0) std::cerr << feitas << " instancias\n";
    }
    std::cerr << "fim: " << feitas << " instancias, " << divergencias << " divergencias\n";
    return divergencias == 0 ? 0 : 2;
}

int modo_exato(const std::vector<std::string>& pastas, const std::string& saida, int max_clientes,
               bool conferir_split) {
    std::ofstream csv(saida);
    csv << "conjunto,instancia,n,alpha,otimo_exato,ciclos_enumerados,divergencias_split,segundos,melhor_ciclo\n";
    csv << std::setprecision(12);
    long long total_divergencias = 0;
    int feitas = 0;
    for (const ArquivoInstancia& arq : listar_instancias(pastas)) {
        Instancia inst = ler_instancia(arq.caminho, arq.conjunto, arq.nome);
        if (inst.num_clientes > max_clientes) continue;
        auto inicio = Relogio::now();
        ResultadoExato res = otimo_por_enumeracao(inst, conferir_split);
        double segundos = std::chrono::duration<double>(Relogio::now() - inicio).count();
        total_divergencias += res.divergencias_split;

        std::string ciclo_texto;
        for (size_t a = 0; a < res.melhor_ciclo.size(); ++a)
            ciclo_texto += (a ? "-" : "") + std::to_string(res.melhor_ciclo[a]);
        csv << inst.conjunto << ',' << inst.nome << ',' << inst.num_clientes << ',' << inst.alpha << ','
            << res.otimo << ',' << res.ciclos_enumerados << ',' << res.divergencias_split << ','
            << std::fixed << std::setprecision(3) << segundos << std::defaultfloat
            << std::setprecision(12) << ',' << ciclo_texto << '\n';
        ++feitas;
    }
    std::cerr << "fim: " << feitas << " instancias, " << total_divergencias
              << " divergencias Split x SplitLazy\n";
    return total_divergencias == 0 ? 0 : 2;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "uso: fstsp experimento|exato --pasta DIR --saida ARQUIVO.csv\n";
        return 1;
    }
    std::string modo = argv[1], saida;
    std::vector<std::string> pastas;
    double tempo_min_ms = 20.0;
    int max_clientes = 9;
    unsigned semente = 2026;
    bool conferir_split = false;

    for (int a = 2; a < argc; ++a) {
        std::string op = argv[a];
        auto valor = [&]() { return std::string(a + 1 < argc ? argv[++a] : ""); };
        if (op == "--pasta") pastas.push_back(valor());
        else if (op == "--saida") saida = valor();
        else if (op == "--tempo-min-ms") tempo_min_ms = std::stod(valor());
        else if (op == "--max-clientes") max_clientes = std::stoi(valor());
        else if (op == "--semente") semente = (unsigned)std::stoul(valor());
        else if (op == "--conferir-split") conferir_split = true;
        else { std::cerr << "opcao desconhecida: " << op << "\n"; return 1; }
    }

    if (modo == "experimento") return modo_experimento(pastas, saida, tempo_min_ms, semente);
    if (modo == "exato") return modo_exato(pastas, saida, max_clientes, conferir_split);
    std::cerr << "modo desconhecido: " << modo << "\n";
    return 1;
}
