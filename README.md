# Split e SplitLazy no FSTSP

Código do TCC 2. Aplica o *Split* e o *SplitLazy* (Hokama, Lintzmayer e San Felice, 2024) ao ciclo
ótimo do TSP e ao ciclo do Vizinho Mais Próximo, nas instâncias de Bouman, Agatz e Schmidt (2018),
e calcula o ótimo exato do FSTSP por enumeração nas instâncias com até 10 nós.

## Arquivos

- `src/instancia.hpp`: leitura das instâncias e dos ciclos, Vizinho Mais Próximo e 1-localidade.
- `src/algoritmos.hpp`: *Split*, *SplitLazy*, avaliação independente da solução e enumeração exata.
- `src/main.cpp`: modos `experimento` e `exato`, medição de tempo e saída em CSV.

## Compilar

```bash
g++ -O2 -std=c++17 -static -o fstsp src/main.cpp
```

## Executar

`D` é a pasta das instâncias de <https://github.com/pcbouman-eur/TSP-D-Instances>.

```bash
# Cenários A e B: Split e SplitLazy sobre o ciclo ótimo e o do VMP
./fstsp experimento --pasta "$D/uniform" --pasta "$D/singlecenter" --pasta "$D/doublecenter" \
    --saida resultados.csv --tempo-min-ms 20

# Cenário C: ótimo exato por enumeração (até 9 clientes)
./fstsp exato --pasta "$D/uniform" --pasta "$D/singlecenter" --pasta "$D/doublecenter" \
    --saida exato.csv --max-clientes 9 [--conferir-split]
```

Instâncias sem o arquivo `-tsp.txt` (apenas `uniform-121-n375`) são ignoradas no modo `experimento`.

## Colunas de `resultados.csv`

| Coluna | Significado |
|---|---|
| `n`, `alpha` | número de clientes e velocidade relativa do drone |
| `ciclo` | `otimo` (arquivo `-tsp.txt`) ou `vmp` |
| `comprimento_ciclo` | tempo do caminhão percorrendo o ciclo sozinho |
| `frac_1local` | fração de clientes com 1-localidade |
| `makespan_split`, `makespan_lazy` | valor da solução de cada algoritmo |
| `valido` | 1 se os dois valores são iguais (tolerância relativa 1e-9) e conferem com a avaliação independente |
| `triplas_split`, `triplas_lazy` | triplas avaliadas |
| `t_split_us`, `t_lazy_us` | tempo médio de uma decodificação, em microssegundos |
| `reps_split`, `reps_lazy` | execuções do lote medido |

`exato.csv` traz `otimo_exato`, `ciclos_enumerados` (n!), `divergencias_split` (com `--conferir-split`),
`segundos` e `melhor_ciclo`.
